#include "mj_world.h"

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <mujoco/mujoco.h>

#include <cstring>
#include <type_traits>

using namespace godot;

// Keep the warning-name table aligned with mjtWarning. A new MuJoCo release
// that adds/removes a warning makes this a compile error instead of silently
// mislabeling get_warnings() keys.
static_assert(mjNWARNING == 7, "Update kWarningNames to match mjtWarning in this MuJoCo version");

static PackedFloat64Array copy_mjt_array(const mjtNum *src, int n) {
	PackedFloat64Array out;
	if (src == nullptr || n <= 0) {
		return out;
	}
	out.resize(n);
	double *dst = out.ptrw();
	if constexpr (std::is_same_v<mjtNum, double>) {
		memcpy(dst, src, sizeof(double) * static_cast<size_t>(n));
	} else {
		for (int i = 0; i < n; ++i) {
			dst[i] = static_cast<double>(src[i]);
		}
	}
	return out;
}

static void write_mjt_array(mjtNum *dst, const PackedFloat64Array &values, int n) {
	const double *src = values.ptr();
	if constexpr (std::is_same_v<mjtNum, double>) {
		memcpy(dst, src, sizeof(double) * static_cast<size_t>(n));
	} else {
		for (int i = 0; i < n; ++i) {
			dst[i] = static_cast<mjtNum>(src[i]);
		}
	}
}

static String gmj_norm_rel(const String &p) {
	return p.replace("\\", "/").strip_edges();
}

static bool gmj_is_safe_rel(const String &rel) {
	if (rel.is_empty()) {
		return false;
	}
	return !rel.contains("..");
}

static bool gmj_should_skip_dir(const String &fn) {
	return fn.begins_with(".") || fn == "addons";
}

static bool gmj_should_skip_file(const String &fn) {
	return fn.ends_with(".import") || fn.ends_with(".uid") || fn.ends_with(".so") || fn.ends_with(".dll") ||
	       fn.ends_with(".dylib") || fn.ends_with(".a") || fn.ends_with(".exe") || fn.ends_with(".lib");
}

static bool gmj_looks_like_mjcf(const String &path, const PackedByteArray &bytes) {
	const String ext = path.get_extension().to_lower();
	if (ext == "xml" || ext == "mjcf") {
		return true;
	}
	if (bytes.size() > 0 && bytes[0] == '<') {
		return true;
	}
	return false;
}

static void gmj_find_quoted_attrs(const String &xml, const String &attr, PackedStringArray &out) {
	const String needles[2] = {attr + String("=\""), attr + String("='")};
	const String closers[2] = {String("\""), String("'")};
	for (int n = 0; n < 2; ++n) {
		int from = 0;
		while (true) {
			const int i = xml.find(needles[n], from);
			if (i < 0) {
				break;
			}
			const int start = i + needles[n].length();
			const int end = xml.find(closers[n], start);
			if (end < 0) {
				break;
			}
			const String val = gmj_norm_rel(xml.substr(start, end - start));
			if (!val.is_empty()) {
				out.push_back(val);
			}
			from = end + 1;
		}
	}
}

static bool gmj_vfs_add(mjVFS *vfs, const String &key, const PackedByteArray &bytes, int &added) {
	if (key.is_empty() || bytes.is_empty()) {
		return false;
	}
	const CharString key_cs = key.utf8();
	if (mj_addBufferVFS(vfs, key_cs.get_data(), bytes.ptr(), (int)bytes.size()) == 0) {
		++added;
		return true;
	}
	return false;
}

// Recursively add every *asset-like* file under `dir`, keyed by its path
// relative to the model's base directory. Skips editor/VCS/addon dirs and
// binaries MuJoCo cannot consume.
static void gmj_add_dir_to_vfs(mjVFS *vfs, const String &dir, const String &rel_prefix, int &added) {
	Ref<DirAccess> da = DirAccess::open(dir);
	if (da.is_null()) {
		return;
	}
	da->list_dir_begin();
	String fn = da->get_next();
	while (!fn.is_empty()) {
		if (da->current_is_dir()) {
			if (!gmj_should_skip_dir(fn)) {
				gmj_add_dir_to_vfs(vfs, dir.path_join(fn), rel_prefix + fn + String("/"), added);
			}
		} else if (!gmj_should_skip_file(fn)) {
			const PackedByteArray b = FileAccess::get_file_as_bytes(dir.path_join(fn));
			gmj_vfs_add(vfs, rel_prefix + fn, b, added);
		}
		fn = da->get_next();
	}
	da->list_dir_end();
}

static String gmj_xml_from_bytes(const PackedByteArray &bytes) {
	if (bytes.is_empty()) {
		return String();
	}
	return String::utf8(reinterpret_cast<const char *>(bytes.ptr()), bytes.size());
}

static bool gmj_add_rel_file(mjVFS *vfs, const String &base_dir, const String &rel, int &added,
                             PackedByteArray &out_bytes) {
	if (!gmj_is_safe_rel(rel)) {
		return false;
	}
	const String abs = base_dir.path_join(rel);
	if (!FileAccess::file_exists(abs)) {
		return false;
	}
	out_bytes = FileAccess::get_file_as_bytes(abs);
	if (out_bytes.is_empty()) {
		return false;
	}
	gmj_vfs_add(vfs, rel, out_bytes, added);
	return true;
}

// Walk MJCF for <include file>, compiler meshdir/texturedir/assetdir, and
// generic file="..." assets. Only those referenced paths (plus the contents of
// compiler resource dirs) are copied into the VFS — never the whole project
// tree, so a model sitting next to addons/ or .godot/ stays cheap.
static void gmj_populate_vfs_from_mjcf(mjVFS *vfs, const String &base_dir, const String &rel_xml,
                                       const String &xml_text, String &meshdir, String &texturedir, String &assetdir,
                                       int &added, PackedStringArray &visited) {
	if (rel_xml.is_empty() || visited.has(rel_xml)) {
		return;
	}
	visited.push_back(rel_xml);

	PackedStringArray meshdirs;
	PackedStringArray texturedirs;
	PackedStringArray assetdirs;
	gmj_find_quoted_attrs(xml_text, "meshdir", meshdirs);
	gmj_find_quoted_attrs(xml_text, "texturedir", texturedirs);
	gmj_find_quoted_attrs(xml_text, "assetdir", assetdirs);
	if (meshdirs.size() > 0 && gmj_is_safe_rel(meshdirs[0])) {
		meshdir = meshdirs[0];
	}
	if (texturedirs.size() > 0 && gmj_is_safe_rel(texturedirs[0])) {
		texturedir = texturedirs[0];
	}
	if (assetdirs.size() > 0 && gmj_is_safe_rel(assetdirs[0])) {
		assetdir = assetdirs[0];
	}

	if (!meshdir.is_empty()) {
		gmj_add_dir_to_vfs(vfs, base_dir.path_join(meshdir), meshdir.ends_with("/") ? meshdir : meshdir + "/",
		                   added);
	}
	if (!texturedir.is_empty() && texturedir != meshdir) {
		gmj_add_dir_to_vfs(vfs, base_dir.path_join(texturedir),
		                   texturedir.ends_with("/") ? texturedir : texturedir + "/", added);
	}
	if (!assetdir.is_empty() && assetdir != meshdir && assetdir != texturedir) {
		gmj_add_dir_to_vfs(vfs, base_dir.path_join(assetdir),
		                   assetdir.ends_with("/") ? assetdir : assetdir + "/", added);
	}

	const String xml_dir = rel_xml.get_base_dir();
	PackedStringArray files;
	gmj_find_quoted_attrs(xml_text, "file", files);
	for (int i = 0; i < files.size(); ++i) {
		const String rel = files[i];
		PackedStringArray candidates;
		if (!xml_dir.is_empty()) {
			candidates.push_back(xml_dir.path_join(rel));
		}
		candidates.push_back(rel);
		if (!meshdir.is_empty()) {
			candidates.push_back(meshdir.path_join(rel));
		}
		if (!texturedir.is_empty()) {
			candidates.push_back(texturedir.path_join(rel));
		}
		if (!assetdir.is_empty()) {
			candidates.push_back(assetdir.path_join(rel));
		}

		for (int c = 0; c < candidates.size(); ++c) {
			PackedByteArray bytes;
			if (!gmj_add_rel_file(vfs, base_dir, candidates[c], added, bytes)) {
				continue;
			}
			if (candidates[c] != rel) {
				gmj_vfs_add(vfs, rel, bytes, added);
			}
			if (gmj_looks_like_mjcf(candidates[c], bytes)) {
				gmj_populate_vfs_from_mjcf(vfs, base_dir, candidates[c], gmj_xml_from_bytes(bytes),
				                           meshdir, texturedir, assetdir, added, visited);
			}
			break;
		}
	}
}

MjWorld::MjWorld() {}

MjWorld::~MjWorld() {
	free_model();
}

void MjWorld::free_model() {
	if (data != nullptr) {
		mj_deleteData(data);
		data = nullptr;
	}
	if (model != nullptr) {
		mj_deleteModel(model);
		model = nullptr;
	}
	last_error = "";
	last_vfs_files = 0;
}

bool MjWorld::fail_load(const String &msg) {
	last_error = msg;
	UtilityFunctions::push_error("MjWorld: " + last_error);
	emit_signal("load_failed", last_error);
	return false;
}

bool MjWorld::commit_vfs_model(mjVFS_ *vfs, const String &main_name) {
	const CharString main_cs = main_name.utf8();
	char error[1024] = {0};
	mjModel *new_model = mj_loadXML(main_cs.get_data(), vfs, error, sizeof(error));
	mj_deleteVFS(vfs);
	if (new_model == nullptr) {
		// Keep any currently loaded model intact (load-then-swap).
		return fail_load(String::utf8(error));
	}

	mjData *new_data = mj_makeData(new_model);
	if (new_data == nullptr) {
		mj_deleteModel(new_model);
		return fail_load("failed to allocate mjData");
	}

	// Enable energy computation and run forward once so kinematics, sensors,
	// COM and energy are valid on the first frame (before any step()).
	new_model->opt.enableflags |= mjENBL_ENERGY;
	mj_forward(new_model, new_data);

	// Success: swap in the new model/data and release the old. Preserve the
	// VFS file count from this load — free_model() would otherwise zero it.
	const int vfs_keep = last_vfs_files;
	free_model();
	model = new_model;
	data = new_data;
	last_error = "";
	last_vfs_files = vfs_keep;
	emit_signal("model_loaded");
	return true;
}

bool MjWorld::load_model(const String &xml_path) {
	if (xml_path.is_empty()) {
		last_vfs_files = 0;
		return fail_load("model path is empty");
	}

	// Read through Godot's filesystem rather than a raw OS path so res:// works
	// in an exported game (packed into the PCK), not only in the editor.
	const PackedByteArray bytes = FileAccess::get_file_as_bytes(xml_path);
	if (bytes.is_empty()) {
		last_vfs_files = 0;
		return fail_load("could not read model file: " + xml_path);
	}

	String main_name = xml_path.get_file();
	if (main_name.is_empty()) {
		main_name = "model.xml";
	}

	mjVFS vfs;
	mj_defaultVFS(&vfs);
	int added = 0;
	gmj_vfs_add(&vfs, main_name, bytes, added);

	const String base_dir = xml_path.get_base_dir();
	if (!base_dir.is_empty()) {
		String meshdir;
		String texturedir;
		String assetdir;
		PackedStringArray visited;
		gmj_populate_vfs_from_mjcf(&vfs, base_dir, main_name, gmj_xml_from_bytes(bytes), meshdir, texturedir,
		                           assetdir, added, visited);
	}

	last_vfs_files = added;
	return commit_vfs_model(&vfs, main_name);
}

bool MjWorld::load_model_from_string(const String &xml_text, const String &virtual_name) {
	if (xml_text.is_empty()) {
		last_vfs_files = 0;
		return fail_load("model string is empty");
	}
	const String name = virtual_name.is_empty() ? String("model.xml") : virtual_name;
	return load_model_from_buffer(xml_text.to_utf8_buffer(), name);
}

bool MjWorld::load_model_from_buffer(const PackedByteArray &bytes, const String &vfs_name) {
	// Single-buffer load (no sibling assets), used for in-memory strings.
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	int added = 0;
	if (!gmj_vfs_add(&vfs, vfs_name, bytes, added)) {
		mj_deleteVFS(&vfs);
		last_vfs_files = 0;
		return fail_load("mj_addBufferVFS failed");
	}
	last_vfs_files = added;
	return commit_vfs_model(&vfs, vfs_name);
}

bool MjWorld::is_ready() const {
	return model != nullptr && data != nullptr;
}

void MjWorld::reset() {
	if (!is_ready()) {
		return;
	}
	mj_resetData(model, data);
	// Recompute derived quantities so queries are valid immediately after reset.
	mj_forward(model, data);
}

bool MjWorld::step(int n) {
	if (!is_ready()) {
		last_error = "world is not ready";
		return false;
	}
	if (n <= 0) {
		return true; // no-op for zero/negative counts
	}
	for (int i = 0; i < n; ++i) {
		mj_step(model, data);
	}
	return true;
}

void MjWorld::forward() {
	if (!is_ready()) {
		return;
	}
	mj_forward(model, data);
}

int MjWorld::get_nq() const {
	return is_ready() ? model->nq : -1;
}

int MjWorld::get_nv() const {
	return is_ready() ? model->nv : -1;
}

int MjWorld::get_nu() const {
	return is_ready() ? model->nu : -1;
}

int MjWorld::get_nbody() const {
	return is_ready() ? model->nbody : -1;
}

int MjWorld::get_njnt() const {
	return is_ready() ? model->njnt : -1;
}

int MjWorld::get_nsensor() const {
	return is_ready() ? model->nsensor : -1;
}

double MjWorld::get_time() const {
	return is_ready() ? (double)data->time : 0.0;
}

double MjWorld::get_timestep() const {
	return is_ready() ? (double)model->opt.timestep : 0.0;
}

void MjWorld::set_timestep(double dt) {
	if (is_ready() && dt > 0.0) {
		model->opt.timestep = (mjtNum)dt;
	}
}

int MjWorld::body_id(const String &name) const {
	if (!is_ready()) {
		return -1;
	}
	return mj_name2id(model, mjOBJ_BODY, name.utf8().get_data());
}

int MjWorld::joint_id(const String &name) const {
	if (!is_ready()) {
		return -1;
	}
	return mj_name2id(model, mjOBJ_JOINT, name.utf8().get_data());
}

int MjWorld::actuator_id(const String &name) const {
	if (!is_ready()) {
		return -1;
	}
	return mj_name2id(model, mjOBJ_ACTUATOR, name.utf8().get_data());
}

int MjWorld::sensor_id(const String &name) const {
	if (!is_ready()) {
		return -1;
	}
	return mj_name2id(model, mjOBJ_SENSOR, name.utf8().get_data());
}

String MjWorld::body_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->nbody) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_BODY, id);
	return name != nullptr ? String::utf8(name) : String();
}

String MjWorld::joint_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->njnt) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_JOINT, id);
	return name != nullptr ? String::utf8(name) : String();
}

String MjWorld::actuator_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->nu) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_ACTUATOR, id);
	return name != nullptr ? String::utf8(name) : String();
}

String MjWorld::sensor_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->nsensor) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_SENSOR, id);
	return name != nullptr ? String::utf8(name) : String();
}

bool MjWorld::set_ctrl(int index, double value) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_ctrl: " + last_error);
		return false;
	}
	if (index < 0 || index >= model->nu) {
		last_error = "actuator index out of range: " + String::num_int64(index);
		UtilityFunctions::push_error("MjWorld.set_ctrl: " + last_error);
		return false;
	}
	data->ctrl[index] = (mjtNum)value;
	return true;
}

double MjWorld::get_ctrl(int index) const {
	if (!is_ready() || index < 0 || index >= model->nu) {
		return 0.0;
	}
	return (double)data->ctrl[index];
}

PackedFloat64Array MjWorld::get_qpos() const {
	if (!is_ready()) {
		return PackedFloat64Array();
	}
	return copy_mjt_array(data->qpos, model->nq);
}

bool MjWorld::set_qpos(const PackedFloat64Array &values) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_qpos: " + last_error);
		return false;
	}
	if ((int)values.size() != model->nq) {
		last_error = "set_qpos expects " + String::num_int64(model->nq) + " values, got " +
		             String::num_int64(values.size());
		UtilityFunctions::push_error("MjWorld.set_qpos: " + last_error);
		return false;
	}
	write_mjt_array(data->qpos, values, model->nq);
	return true;
}

PackedFloat64Array MjWorld::get_qvel() const {
	if (!is_ready()) {
		return PackedFloat64Array();
	}
	return copy_mjt_array(data->qvel, model->nv);
}

bool MjWorld::set_qvel(const PackedFloat64Array &values) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_qvel: " + last_error);
		return false;
	}
	if ((int)values.size() != model->nv) {
		last_error = "set_qvel expects " + String::num_int64(model->nv) + " values, got " +
		             String::num_int64(values.size());
		UtilityFunctions::push_error("MjWorld.set_qvel: " + last_error);
		return false;
	}
	write_mjt_array(data->qvel, values, model->nv);
	return true;
}

PackedFloat64Array MjWorld::get_ctrl_array() const {
	if (!is_ready()) {
		return PackedFloat64Array();
	}
	return copy_mjt_array(data->ctrl, model->nu);
}

bool MjWorld::set_ctrl_array(const PackedFloat64Array &values) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_ctrl_array: " + last_error);
		return false;
	}
	if ((int)values.size() != model->nu) {
		last_error = "set_ctrl_array expects " + String::num_int64(model->nu) + " values, got " +
		             String::num_int64(values.size());
		UtilityFunctions::push_error("MjWorld.set_ctrl_array: " + last_error);
		return false;
	}
	write_mjt_array(data->ctrl, values, model->nu);
	return true;
}

PackedFloat64Array MjWorld::get_sensordata() const {
	if (!is_ready()) {
		return PackedFloat64Array();
	}
	return copy_mjt_array(data->sensordata, model->nsensordata);
}

PackedFloat64Array MjWorld::get_sensor(int sensor_index) const {
	if (!is_ready() || sensor_index < 0 || sensor_index >= model->nsensor) {
		return PackedFloat64Array();
	}
	return copy_mjt_array(data->sensordata + model->sensor_adr[sensor_index], model->sensor_dim[sensor_index]);
}

Vector3 MjWorld::body_world_position(int body_index) const {
	if (!is_ready() || body_index < 0 || body_index >= model->nbody) {
		return Vector3();
	}
	const mjtNum *xpos = data->xpos + (3 * body_index);
	return Vector3((float)xpos[0], (float)xpos[1], (float)xpos[2]);
}

Quaternion MjWorld::body_world_quaternion(int body_index) const {
	if (!is_ready() || body_index < 0 || body_index >= model->nbody) {
		return Quaternion();
	}
	// MuJoCo stores quaternions as (w, x, y, z); Godot's Quaternion is (x, y, z, w).
	const mjtNum *q = data->xquat + (4 * body_index);
	return Quaternion((float)q[1], (float)q[2], (float)q[3], (float)q[0]);
}

Transform3D MjWorld::body_world_transform(int body_index) const {
	if (!is_ready() || body_index < 0 || body_index >= model->nbody) {
		return Transform3D();
	}
	return Transform3D(Basis(body_world_quaternion(body_index)), body_world_position(body_index));
}

String MjWorld::get_mujoco_version() const {
	// mj_versionString() returns the full semantic version (e.g. "3.12.0").
	return String::utf8(mj_versionString());
}

String MjWorld::get_last_error() const {
	return last_error;
}

int MjWorld::get_last_vfs_files() const {
	return last_vfs_files;
}

int MjWorld::get_ncon() const {
	return is_ready() ? data->ncon : -1;
}

double MjWorld::get_potential_energy() const {
	return is_ready() ? (double)data->energy[0] : 0.0;
}

double MjWorld::get_kinetic_energy() const {
	return is_ready() ? (double)data->energy[1] : 0.0;
}

Dictionary MjWorld::get_warnings() const {
	static const char *kWarningNames[mjNWARNING] = {"INERTIA", "CONTACTFULL", "CNSTRFULL", "BADQPOS",
	                                                "BADQVEL", "BADQACC",     "BADCTRL"};
	Dictionary out;
	if (!is_ready()) {
		return out;
	}
	for (int i = 0; i < mjNWARNING; ++i) {
		const int count = data->warning[i].number;
		if (count > 0) {
			out[String(kWarningNames[i])] = count;
		}
	}
	return out;
}

bool MjWorld::has_warnings() const {
	if (!is_ready()) {
		return false;
	}
	for (int i = 0; i < mjNWARNING; ++i) {
		if (data->warning[i].number > 0) {
			return true;
		}
	}
	return false;
}

Dictionary MjWorld::get_debug_info() const {
	Dictionary info;
	info["mujoco_version"] = get_mujoco_version();
	info["ready"] = is_ready();
	info["last_error"] = last_error;
	info["vfs_files"] = last_vfs_files;
	if (!is_ready()) {
		return info;
	}

	info["time"] = get_time();
	info["timestep"] = get_timestep();
	info["nq"] = model->nq;
	info["nv"] = model->nv;
	info["nu"] = model->nu;
	info["nbody"] = model->nbody;
	info["njnt"] = model->njnt;
	info["nsensor"] = model->nsensor;
	info["ncon"] = data->ncon;

	Dictionary energy;
	const double potential = (double)data->energy[0];
	const double kinetic = (double)data->energy[1];
	energy["potential"] = potential;
	energy["kinetic"] = kinetic;
	energy["total"] = potential + kinetic;
	info["energy"] = energy;

	info["warnings"] = get_warnings();
	return info;
}

Array MjWorld::get_contacts() const {
	Array out;
	if (!is_ready()) {
		return out;
	}
	for (int i = 0; i < data->ncon; ++i) {
		const mjContact &c = data->contact[i];
		mjtNum wrench[6] = {0};
		mj_contactForce(model, data, i, wrench);
		// c.frame is a 3x3 matrix whose rows are the contact axes in world
		// coordinates (row 0 = normal). The contact force is expressed in that
		// frame, so rotate it back into world space.
		const Vector3 n((float)c.frame[0], (float)c.frame[1], (float)c.frame[2]);
		const Vector3 t1((float)c.frame[3], (float)c.frame[4], (float)c.frame[5]);
		const Vector3 t2((float)c.frame[6], (float)c.frame[7], (float)c.frame[8]);
		const Vector3 force = n * (float)wrench[0] + t1 * (float)wrench[1] + t2 * (float)wrench[2];

		Dictionary d;
		d["pos"] = Vector3((float)c.pos[0], (float)c.pos[1], (float)c.pos[2]);
		d["normal"] = n;
		d["force"] = force;
		d["distance"] = (double)c.dist;
		out.push_back(d);
	}
	return out;
}

Vector3 MjWorld::get_center_of_mass() const {
	if (!is_ready()) {
		return Vector3();
	}
	// subtree_com of the world body (index 0) is the whole-model center of mass.
	const mjtNum *com = data->subtree_com;
	return Vector3((float)com[0], (float)com[1], (float)com[2]);
}

Vector3 MjWorld::get_joint_anchor(int joint_index) const {
	if (!is_ready() || joint_index < 0 || joint_index >= model->njnt) {
		return Vector3();
	}
	const mjtNum *a = data->xanchor + (3 * joint_index);
	return Vector3((float)a[0], (float)a[1], (float)a[2]);
}

Vector3 MjWorld::get_joint_axis(int joint_index) const {
	if (!is_ready() || joint_index < 0 || joint_index >= model->njnt) {
		return Vector3();
	}
	const mjtNum *a = data->xaxis + (3 * joint_index);
	return Vector3((float)a[0], (float)a[1], (float)a[2]);
}

void MjWorld::sync_physics_process() {
	const bool enable = auto_step && is_inside_tree() && !Engine::get_singleton()->is_editor_hint();
	set_physics_process(enable);
}

void MjWorld::set_model_path(const String &p_path) {
	const bool changed = model_path != p_path;
	model_path = p_path;
	if (!changed) {
		return;
	}
	if (is_inside_tree() && !Engine::get_singleton()->is_editor_hint() && !model_path.is_empty()) {
		load_model(model_path);
	}
}

String MjWorld::get_model_path() const {
	return model_path;
}

void MjWorld::set_steps_per_tick(int p_steps) {
	steps_per_tick = p_steps < 1 ? 1 : p_steps;
}

int MjWorld::get_steps_per_tick() const {
	return steps_per_tick;
}

void MjWorld::set_auto_step(bool p_enabled) {
	auto_step = p_enabled;
	sync_physics_process();
}

bool MjWorld::get_auto_step() const {
	return auto_step;
}

void MjWorld::_ready() {
	// Do not touch the simulation while running inside the editor.
	if (Engine::get_singleton()->is_editor_hint()) {
		set_physics_process(false);
		return;
	}
	sync_physics_process();
	if (!model_path.is_empty() && !is_ready()) {
		load_model(model_path);
	}
}

void MjWorld::_physics_process(double delta) {
	(void)delta;
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (auto_step && is_ready()) {
		step(steps_per_tick);
	}
}

void MjWorld::_bind_methods() {
	ClassDB::bind_method(D_METHOD("load_model", "xml_path"), &MjWorld::load_model);
	ClassDB::bind_method(D_METHOD("load_model_from_string", "xml_text", "virtual_name"),
	                     &MjWorld::load_model_from_string, DEFVAL(String()));
	ClassDB::bind_method(D_METHOD("free_model"), &MjWorld::free_model);
	ClassDB::bind_method(D_METHOD("is_ready"), &MjWorld::is_ready);
	ClassDB::bind_method(D_METHOD("reset"), &MjWorld::reset);
	ClassDB::bind_method(D_METHOD("step", "n"), &MjWorld::step, DEFVAL(1));
	ClassDB::bind_method(D_METHOD("forward"), &MjWorld::forward);

	ClassDB::bind_method(D_METHOD("get_nq"), &MjWorld::get_nq);
	ClassDB::bind_method(D_METHOD("get_nv"), &MjWorld::get_nv);
	ClassDB::bind_method(D_METHOD("get_nu"), &MjWorld::get_nu);
	ClassDB::bind_method(D_METHOD("get_nbody"), &MjWorld::get_nbody);
	ClassDB::bind_method(D_METHOD("get_njnt"), &MjWorld::get_njnt);
	ClassDB::bind_method(D_METHOD("get_nsensor"), &MjWorld::get_nsensor);

	ClassDB::bind_method(D_METHOD("get_time"), &MjWorld::get_time);
	ClassDB::bind_method(D_METHOD("get_timestep"), &MjWorld::get_timestep);
	ClassDB::bind_method(D_METHOD("set_timestep", "dt"), &MjWorld::set_timestep);

	ClassDB::bind_method(D_METHOD("body_id", "name"), &MjWorld::body_id);
	ClassDB::bind_method(D_METHOD("joint_id", "name"), &MjWorld::joint_id);
	ClassDB::bind_method(D_METHOD("actuator_id", "name"), &MjWorld::actuator_id);
	ClassDB::bind_method(D_METHOD("sensor_id", "name"), &MjWorld::sensor_id);
	ClassDB::bind_method(D_METHOD("body_name", "id"), &MjWorld::body_name);
	ClassDB::bind_method(D_METHOD("joint_name", "id"), &MjWorld::joint_name);
	ClassDB::bind_method(D_METHOD("actuator_name", "id"), &MjWorld::actuator_name);
	ClassDB::bind_method(D_METHOD("sensor_name", "id"), &MjWorld::sensor_name);

	ClassDB::bind_method(D_METHOD("set_ctrl", "index", "value"), &MjWorld::set_ctrl);
	ClassDB::bind_method(D_METHOD("get_ctrl", "index"), &MjWorld::get_ctrl);
	ClassDB::bind_method(D_METHOD("get_qpos"), &MjWorld::get_qpos);
	ClassDB::bind_method(D_METHOD("set_qpos", "values"), &MjWorld::set_qpos);
	ClassDB::bind_method(D_METHOD("get_qvel"), &MjWorld::get_qvel);
	ClassDB::bind_method(D_METHOD("set_qvel", "values"), &MjWorld::set_qvel);
	ClassDB::bind_method(D_METHOD("get_ctrl_array"), &MjWorld::get_ctrl_array);
	ClassDB::bind_method(D_METHOD("set_ctrl_array", "values"), &MjWorld::set_ctrl_array);

	ClassDB::bind_method(D_METHOD("get_sensordata"), &MjWorld::get_sensordata);
	ClassDB::bind_method(D_METHOD("get_sensor", "sensor_index"), &MjWorld::get_sensor);

	ClassDB::bind_method(D_METHOD("body_world_position", "body_index"), &MjWorld::body_world_position);
	ClassDB::bind_method(D_METHOD("body_world_quaternion", "body_index"), &MjWorld::body_world_quaternion);
	ClassDB::bind_method(D_METHOD("body_world_transform", "body_index"), &MjWorld::body_world_transform);

	ClassDB::bind_method(D_METHOD("get_mujoco_version"), &MjWorld::get_mujoco_version);
	ClassDB::bind_method(D_METHOD("get_last_error"), &MjWorld::get_last_error);
	ClassDB::bind_method(D_METHOD("get_last_vfs_files"), &MjWorld::get_last_vfs_files);

	ClassDB::bind_method(D_METHOD("get_ncon"), &MjWorld::get_ncon);
	ClassDB::bind_method(D_METHOD("get_kinetic_energy"), &MjWorld::get_kinetic_energy);
	ClassDB::bind_method(D_METHOD("get_potential_energy"), &MjWorld::get_potential_energy);
	ClassDB::bind_method(D_METHOD("get_warnings"), &MjWorld::get_warnings);
	ClassDB::bind_method(D_METHOD("has_warnings"), &MjWorld::has_warnings);
	ClassDB::bind_method(D_METHOD("get_debug_info"), &MjWorld::get_debug_info);

	ClassDB::bind_method(D_METHOD("get_contacts"), &MjWorld::get_contacts);
	ClassDB::bind_method(D_METHOD("get_center_of_mass"), &MjWorld::get_center_of_mass);
	ClassDB::bind_method(D_METHOD("get_joint_anchor", "joint_index"), &MjWorld::get_joint_anchor);
	ClassDB::bind_method(D_METHOD("get_joint_axis", "joint_index"), &MjWorld::get_joint_axis);

	ClassDB::bind_method(D_METHOD("set_model_path", "path"), &MjWorld::set_model_path);
	ClassDB::bind_method(D_METHOD("get_model_path"), &MjWorld::get_model_path);
	ClassDB::bind_method(D_METHOD("set_steps_per_tick", "steps"), &MjWorld::set_steps_per_tick);
	ClassDB::bind_method(D_METHOD("get_steps_per_tick"), &MjWorld::get_steps_per_tick);
	ClassDB::bind_method(D_METHOD("set_auto_step", "enabled"), &MjWorld::set_auto_step);
	ClassDB::bind_method(D_METHOD("get_auto_step"), &MjWorld::get_auto_step);

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "model_path", PROPERTY_HINT_FILE, "*.xml,*.mjcf"), "set_model_path",
	             "get_model_path");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "steps_per_tick"), "set_steps_per_tick", "get_steps_per_tick");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_step"), "set_auto_step", "get_auto_step");

	ADD_SIGNAL(MethodInfo("model_loaded"));
	ADD_SIGNAL(MethodInfo("load_failed", PropertyInfo(Variant::STRING, "error")));
}
