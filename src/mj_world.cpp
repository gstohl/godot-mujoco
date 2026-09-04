#include "mj_world.h"

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <mujoco/mujoco.h>

using namespace godot;

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
}

// Recursively add every file under `dir` to the VFS, keyed by its path relative
// to the model's base directory, so MJCF <include> files and mesh/texture
// assets resolve. Reads through Godot's filesystem so it works from res://.
static void gmj_add_dir_to_vfs(mjVFS *vfs, const String &dir, const String &rel_prefix, int &added) {
	Ref<DirAccess> da = DirAccess::open(dir);
	if (da.is_null()) {
		return;
	}
	da->list_dir_begin();
	String fn = da->get_next();
	while (!fn.is_empty()) {
		if (da->current_is_dir()) {
			if (fn != "." && fn != "..") {
				gmj_add_dir_to_vfs(vfs, dir.path_join(fn), rel_prefix + fn + String("/"), added);
			}
		} else if (!fn.ends_with(".import") && !fn.ends_with(".uid")) {
			const PackedByteArray b = FileAccess::get_file_as_bytes(dir.path_join(fn));
			if (!b.is_empty()) {
				const CharString key = (rel_prefix + fn).utf8();
				if (mj_addBufferVFS(vfs, key.get_data(), b.ptr(), (int)b.size()) == 0) {
					++added;
				}
			}
		}
		fn = da->get_next();
	}
	da->list_dir_end();
}

bool MjWorld::commit_vfs_model(mjVFS_ *vfs, const String &main_name) {
	const CharString main_cs = main_name.utf8();
	char error[1024] = { 0 };
	mjModel *new_model = mj_loadXML(main_cs.get_data(), vfs, error, sizeof(error));
	mj_deleteVFS(vfs);
	if (new_model == nullptr) {
		// Keep any currently loaded model intact (load-then-swap).
		last_error = String::utf8(error);
		UtilityFunctions::push_error("MjWorld: failed to load model: " + last_error);
		return false;
	}

	mjData *new_data = mj_makeData(new_model);
	if (new_data == nullptr) {
		mj_deleteModel(new_model);
		last_error = "failed to allocate mjData";
		UtilityFunctions::push_error("MjWorld: " + last_error);
		return false;
	}

	// Enable energy computation and run forward once so kinematics, sensors,
	// COM and energy are valid on the first frame (before any step()).
	new_model->opt.enableflags |= mjENBL_ENERGY;
	mj_forward(new_model, new_data);

	// Success: swap in the new model/data and release the old.
	free_model();
	model = new_model;
	data = new_data;
	last_error = "";
	return true;
}

bool MjWorld::load_model(const String &xml_path) {
	if (xml_path.is_empty()) {
		last_error = "model path is empty";
		UtilityFunctions::push_error("MjWorld: " + last_error);
		return false;
	}

	// Read through Godot's filesystem rather than a raw OS path so res:// works
	// in an exported game (packed into the PCK), not only in the editor.
	const PackedByteArray bytes = FileAccess::get_file_as_bytes(xml_path);
	if (bytes.is_empty()) {
		last_error = "could not read model file: " + xml_path;
		UtilityFunctions::push_error("MjWorld: " + last_error);
		return false;
	}

	String main_name = xml_path.get_file();
	if (main_name.is_empty()) {
		main_name = "model.xml";
	}

	// Populate the VFS with sibling files from the model's directory so MJCF
	// <include> files and mesh/texture assets resolve; then ensure the main
	// file is present under its name (duplicate adds are ignored).
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	const String base_dir = xml_path.get_base_dir();
	int added = 0;
	if (!base_dir.is_empty()) {
		gmj_add_dir_to_vfs(&vfs, base_dir, "", added);
	}
	const CharString main_cs = main_name.utf8();
	mj_addBufferVFS(&vfs, main_cs.get_data(), bytes.ptr(), (int)bytes.size());

	return commit_vfs_model(&vfs, main_name);
}

bool MjWorld::load_model_from_string(const String &xml_text, const String &virtual_name) {
	if (xml_text.is_empty()) {
		last_error = "model string is empty";
		UtilityFunctions::push_error("MjWorld: " + last_error);
		return false;
	}
	const String name = virtual_name.is_empty() ? String("model.xml") : virtual_name;
	return load_model_from_buffer(xml_text.to_utf8_buffer(), name);
}

bool MjWorld::load_model_from_buffer(const PackedByteArray &bytes, const String &vfs_name) {
	// Single-buffer load (no sibling assets), used for in-memory strings.
	mjVFS vfs;
	mj_defaultVFS(&vfs);
	const CharString name_cs = vfs_name.utf8();
	const int add_rc = mj_addBufferVFS(&vfs, name_cs.get_data(), bytes.ptr(), (int)bytes.size());
	if (add_rc != 0) {
		mj_deleteVFS(&vfs);
		last_error = "mj_addBufferVFS failed (code " + String::num_int64(add_rc) + ")";
		UtilityFunctions::push_error("MjWorld: " + last_error);
		return false;
	}
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
	PackedFloat64Array out;
	if (!is_ready()) {
		return out;
	}
	out.resize(model->nq);
	for (int i = 0; i < model->nq; ++i) {
		out.set(i, (double)data->qpos[i]);
	}
	return out;
}

bool MjWorld::set_qpos(const PackedFloat64Array &values) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_qpos: " + last_error);
		return false;
	}
	if ((int)values.size() != model->nq) {
		last_error = "set_qpos expects " + String::num_int64(model->nq) + " values, got " + String::num_int64(values.size());
		UtilityFunctions::push_error("MjWorld.set_qpos: " + last_error);
		return false;
	}
	for (int i = 0; i < model->nq; ++i) {
		data->qpos[i] = (mjtNum)values[i];
	}
	return true;
}

PackedFloat64Array MjWorld::get_qvel() const {
	PackedFloat64Array out;
	if (!is_ready()) {
		return out;
	}
	out.resize(model->nv);
	for (int i = 0; i < model->nv; ++i) {
		out.set(i, (double)data->qvel[i]);
	}
	return out;
}

bool MjWorld::set_qvel(const PackedFloat64Array &values) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_qvel: " + last_error);
		return false;
	}
	if ((int)values.size() != model->nv) {
		last_error = "set_qvel expects " + String::num_int64(model->nv) + " values, got " + String::num_int64(values.size());
		UtilityFunctions::push_error("MjWorld.set_qvel: " + last_error);
		return false;
	}
	for (int i = 0; i < model->nv; ++i) {
		data->qvel[i] = (mjtNum)values[i];
	}
	return true;
}

PackedFloat64Array MjWorld::get_ctrl_array() const {
	PackedFloat64Array out;
	if (!is_ready()) {
		return out;
	}
	out.resize(model->nu);
	for (int i = 0; i < model->nu; ++i) {
		out.set(i, (double)data->ctrl[i]);
	}
	return out;
}

bool MjWorld::set_ctrl_array(const PackedFloat64Array &values) {
	if (!is_ready()) {
		last_error = "world is not ready";
		UtilityFunctions::push_error("MjWorld.set_ctrl_array: " + last_error);
		return false;
	}
	if ((int)values.size() != model->nu) {
		last_error = "set_ctrl_array expects " + String::num_int64(model->nu) + " values, got " + String::num_int64(values.size());
		UtilityFunctions::push_error("MjWorld.set_ctrl_array: " + last_error);
		return false;
	}
	for (int i = 0; i < model->nu; ++i) {
		data->ctrl[i] = (mjtNum)values[i];
	}
	return true;
}

PackedFloat64Array MjWorld::get_sensordata() const {
	PackedFloat64Array out;
	if (!is_ready()) {
		return out;
	}
	out.resize(model->nsensordata);
	for (int i = 0; i < model->nsensordata; ++i) {
		out.set(i, (double)data->sensordata[i]);
	}
	return out;
}

PackedFloat64Array MjWorld::get_sensor(int sensor_index) const {
	PackedFloat64Array out;
	if (!is_ready() || sensor_index < 0 || sensor_index >= model->nsensor) {
		return out;
	}
	const int adr = model->sensor_adr[sensor_index];
	const int dim = model->sensor_dim[sensor_index];
	out.resize(dim);
	for (int i = 0; i < dim; ++i) {
		out.set(i, (double)data->sensordata[adr + i]);
	}
	return out;
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
	static const char *kWarningNames[mjNWARNING] = {
		"INERTIA", "CONTACTFULL", "CNSTRFULL",
		"BADQPOS", "BADQVEL", "BADQACC", "BADCTRL"
	};
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
		mjtNum wrench[6] = { 0 };
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

void MjWorld::set_model_path(const String &p_path) {
	model_path = p_path;
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
}

bool MjWorld::get_auto_step() const {
	return auto_step;
}

void MjWorld::_ready() {
	// Do not touch the simulation while running inside the editor.
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (!model_path.is_empty()) {
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
	ClassDB::bind_method(D_METHOD("load_model_from_string", "xml_text", "virtual_name"), &MjWorld::load_model_from_string, DEFVAL(String()));
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

	ADD_PROPERTY(PropertyInfo(Variant::STRING, "model_path", PROPERTY_HINT_FILE, "*.xml,*.mjcf"), "set_model_path", "get_model_path");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "steps_per_tick"), "set_steps_per_tick", "get_steps_per_tick");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_step"), "set_auto_step", "get_auto_step");
}
