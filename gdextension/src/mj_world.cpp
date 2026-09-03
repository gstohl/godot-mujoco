#include "mj_world.h"

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/core/class_db.hpp>
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

bool MjWorld::load_model(const String &xml_path) {
	free_model();

	if (xml_path.is_empty()) {
		last_error = "model path is empty";
		UtilityFunctions::push_error("MjWorld: " + last_error);
		return false;
	}

	// Resolve res:// or user:// paths to an absolute filesystem path so MuJoCo
	// can open them directly.
	const String absolute = ProjectSettings::get_singleton()->globalize_path(xml_path);

	char error[1024] = { 0 };
	model = mj_loadXML(absolute.utf8().get_data(), nullptr, error, sizeof(error));
	if (model == nullptr) {
		last_error = String(error);
		UtilityFunctions::push_error("MjWorld: failed to load model: " + last_error);
		return false;
	}

	data = mj_makeData(model);
	if (data == nullptr) {
		last_error = "failed to allocate mjData";
		UtilityFunctions::push_error("MjWorld: " + last_error);
		mj_deleteModel(model);
		model = nullptr;
		return false;
	}

	last_error = "";
	return true;
}

bool MjWorld::is_ready() const {
	return model != nullptr && data != nullptr;
}

void MjWorld::reset() {
	if (!is_ready()) {
		return;
	}
	mj_resetData(model, data);
}

bool MjWorld::step(int n) {
	if (!is_ready()) {
		last_error = "world is not ready";
		return false;
	}
	if (n < 1) {
		n = 1;
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

String MjWorld::body_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->nbody) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_BODY, id);
	return name != nullptr ? String(name) : String();
}

String MjWorld::joint_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->njnt) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_JOINT, id);
	return name != nullptr ? String(name) : String();
}

String MjWorld::actuator_name(int id) const {
	if (!is_ready() || id < 0 || id >= model->nu) {
		return String();
	}
	const char *name = mj_id2name(model, mjOBJ_ACTUATOR, id);
	return name != nullptr ? String(name) : String();
}

void MjWorld::set_ctrl(int index, double value) {
	if (!is_ready() || index < 0 || index >= model->nu) {
		return;
	}
	data->ctrl[index] = (mjtNum)value;
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

void MjWorld::set_qpos(const PackedFloat64Array &values) {
	if (!is_ready()) {
		return;
	}
	const int count = MIN((int)values.size(), model->nq);
	for (int i = 0; i < count; ++i) {
		data->qpos[i] = (mjtNum)values[i];
	}
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

void MjWorld::set_qvel(const PackedFloat64Array &values) {
	if (!is_ready()) {
		return;
	}
	const int count = MIN((int)values.size(), model->nv);
	for (int i = 0; i < count; ++i) {
		data->qvel[i] = (mjtNum)values[i];
	}
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

void MjWorld::set_ctrl_array(const PackedFloat64Array &values) {
	if (!is_ready()) {
		return;
	}
	const int count = MIN((int)values.size(), model->nu);
	for (int i = 0; i < count; ++i) {
		data->ctrl[i] = (mjtNum)values[i];
	}
}

Vector3 MjWorld::body_world_position(int body_index) const {
	if (!is_ready() || body_index < 0 || body_index >= model->nbody) {
		return Vector3();
	}
	const mjtNum *xpos = data->xpos + (3 * body_index);
	return Vector3((float)xpos[0], (float)xpos[1], (float)xpos[2]);
}

String MjWorld::get_mujoco_version() const {
	const int ver = mj_version();
	const int major = ver / 100;
	const int minor = ver % 100;
	return String::num_int64(major) + "." + String::num_int64(minor);
}

String MjWorld::get_last_error() const {
	return last_error;
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
	ClassDB::bind_method(D_METHOD("free_model"), &MjWorld::free_model);
	ClassDB::bind_method(D_METHOD("is_ready"), &MjWorld::is_ready);
	ClassDB::bind_method(D_METHOD("reset"), &MjWorld::reset);
	ClassDB::bind_method(D_METHOD("step", "n"), &MjWorld::step, DEFVAL(1));
	ClassDB::bind_method(D_METHOD("forward"), &MjWorld::forward);

	ClassDB::bind_method(D_METHOD("get_nq"), &MjWorld::get_nq);
	ClassDB::bind_method(D_METHOD("get_nv"), &MjWorld::get_nv);
	ClassDB::bind_method(D_METHOD("get_nu"), &MjWorld::get_nu);
	ClassDB::bind_method(D_METHOD("get_nbody"), &MjWorld::get_nbody);

	ClassDB::bind_method(D_METHOD("body_id", "name"), &MjWorld::body_id);
	ClassDB::bind_method(D_METHOD("joint_id", "name"), &MjWorld::joint_id);
	ClassDB::bind_method(D_METHOD("actuator_id", "name"), &MjWorld::actuator_id);
	ClassDB::bind_method(D_METHOD("body_name", "id"), &MjWorld::body_name);
	ClassDB::bind_method(D_METHOD("joint_name", "id"), &MjWorld::joint_name);
	ClassDB::bind_method(D_METHOD("actuator_name", "id"), &MjWorld::actuator_name);

	ClassDB::bind_method(D_METHOD("set_ctrl", "index", "value"), &MjWorld::set_ctrl);
	ClassDB::bind_method(D_METHOD("get_ctrl", "index"), &MjWorld::get_ctrl);
	ClassDB::bind_method(D_METHOD("get_qpos"), &MjWorld::get_qpos);
	ClassDB::bind_method(D_METHOD("set_qpos", "values"), &MjWorld::set_qpos);
	ClassDB::bind_method(D_METHOD("get_qvel"), &MjWorld::get_qvel);
	ClassDB::bind_method(D_METHOD("set_qvel", "values"), &MjWorld::set_qvel);
	ClassDB::bind_method(D_METHOD("get_ctrl_array"), &MjWorld::get_ctrl_array);
	ClassDB::bind_method(D_METHOD("set_ctrl_array", "values"), &MjWorld::set_ctrl_array);

	ClassDB::bind_method(D_METHOD("body_world_position", "body_index"), &MjWorld::body_world_position);

	ClassDB::bind_method(D_METHOD("get_mujoco_version"), &MjWorld::get_mujoco_version);
	ClassDB::bind_method(D_METHOD("get_last_error"), &MjWorld::get_last_error);

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
