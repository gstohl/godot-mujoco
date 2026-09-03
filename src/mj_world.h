#ifndef GODOT_MUJOCO_MJ_WORLD_H
#define GODOT_MUJOCO_MJ_WORLD_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

// Forward declarations keep MuJoCo headers out of this file; the real types are
// pulled in only by mj_world.cpp.
struct mjModel_;
struct mjData_;

namespace godot {

// MjWorld owns a full MuJoCo simulation (model + data) directly inside the
// engine. No external bridge library and no manual MuJoCo install are needed:
// a prebuilt MuJoCo runtime is fetched at build time and bundled alongside this
// GDExtension.
//
// Threading: an MjWorld holds a single mjModel/mjData pair, which MuJoCo does
// not guard for concurrent access. Use one MjWorld from one thread at a time
// (the same thread that drives its stepping); separate MjWorld instances on
// separate threads are fine.
class MjWorld : public Node {
	GDCLASS(MjWorld, Node)

	mjModel_ *model = nullptr;
	mjData_ *data = nullptr;
	String last_error;

	String model_path;
	int steps_per_tick = 1;
	bool auto_step = false;

	// Builds a model+data from an in-memory MJCF buffer via MuJoCo's VFS and,
	// on success, swaps it in (leaving any existing model intact on failure).
	bool load_model_from_buffer(const PackedByteArray &bytes, const String &vfs_name);

protected:
	static void _bind_methods();

public:
	MjWorld();
	~MjWorld();

	// Lifecycle.
	bool load_model(const String &xml_path);
	bool load_model_from_string(const String &xml_text, const String &virtual_name = String());
	void free_model();
	bool is_ready() const;
	void reset();
	bool step(int n);
	void forward();

	// Model dimensions.
	int get_nq() const;
	int get_nv() const;
	int get_nu() const;
	int get_nbody() const;
	int get_njnt() const;
	int get_nsensor() const;

	// Simulation clock.
	double get_time() const;
	double get_timestep() const;
	void set_timestep(double dt);

	// Name / id lookup.
	int body_id(const String &name) const;
	int joint_id(const String &name) const;
	int actuator_id(const String &name) const;
	int sensor_id(const String &name) const;
	String body_name(int id) const;
	String joint_name(int id) const;
	String actuator_name(int id) const;
	String sensor_name(int id) const;

	// Scalar control access. Setters return false (and push an error) on a bad
	// index rather than silently ignoring the write.
	bool set_ctrl(int index, double value);
	double get_ctrl(int index) const;

	// Batch state access. Setters require an array whose size exactly matches
	// the corresponding dimension (nq/nv/nu); a mismatch returns false and
	// pushes an error instead of silently truncating.
	PackedFloat64Array get_qpos() const;
	bool set_qpos(const PackedFloat64Array &values);
	PackedFloat64Array get_qvel() const;
	bool set_qvel(const PackedFloat64Array &values);
	PackedFloat64Array get_ctrl_array() const;
	bool set_ctrl_array(const PackedFloat64Array &values);

	// Sensor readout.
	PackedFloat64Array get_sensordata() const;
	PackedFloat64Array get_sensor(int sensor_index) const;

	// Kinematics queries (raw MuJoCo world frame; MuJoCo is Z-up).
	Vector3 body_world_position(int body_index) const;
	Quaternion body_world_quaternion(int body_index) const;
	Transform3D body_world_transform(int body_index) const;

	// Diagnostics.
	String get_mujoco_version() const;
	String get_last_error() const;

	// Debugging: live solver / physics introspection.
	int get_ncon() const; // number of active contacts
	double get_kinetic_energy() const;
	double get_potential_energy() const;
	Dictionary get_warnings() const; // { warning_name: count } for non-zero warnings
	bool has_warnings() const;
	Dictionary get_debug_info() const; // aggregate snapshot for logging/inspection

	// Visual-debug geometry (MuJoCo world frame; feed a MjDebugDraw overlay).
	Array get_contacts() const; // [{ pos, normal, force, distance }]
	Vector3 get_center_of_mass() const; // whole-model COM
	Vector3 get_joint_anchor(int joint_index) const;
	Vector3 get_joint_axis(int joint_index) const;

	// Inspector-exposed properties.
	void set_model_path(const String &p_path);
	String get_model_path() const;
	void set_steps_per_tick(int p_steps);
	int get_steps_per_tick() const;
	void set_auto_step(bool p_enabled);
	bool get_auto_step() const;

	void _ready() override;
	void _physics_process(double delta) override;
};

} // namespace godot

#endif // GODOT_MUJOCO_MJ_WORLD_H
