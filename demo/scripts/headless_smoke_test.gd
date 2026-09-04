extends Node

# Headless end-to-end proof that MuJoCo runs fully inside Godot through the
# GDExtension, with no bridge library and no external MuJoCo install.
# Run with:
#   godot --headless --path demo res://HeadlessTest.tscn

func _ready() -> void:
	var world := MjWorld.new()
	add_child(world)

	print("MuJoCo version reported by extension: ", world.get_mujoco_version())

	if not world.load_model("res://models/pendulum.xml"):
		push_error("load_model failed: " + world.get_last_error())
		get_tree().quit(1)
		return

	print("Model loaded. nq=%d nv=%d nu=%d nbody=%d njnt=%d nsensor=%d" % [
		world.get_nq(), world.get_nv(), world.get_nu(), world.get_nbody(),
		world.get_njnt(), world.get_nsensor()])
	print("timestep=%.4fs  start_time=%.4fs" % [world.get_timestep(), world.get_time()])

	var body_id := world.body_id("pendulum")
	var hinge_id := world.joint_id("hinge")
	var motor_id := world.actuator_id("hinge_motor")
	var sensor_id := world.sensor_id("hinge_pos")
	print("resolved ids: body=%d hinge=%d motor=%d sensor(hinge_pos)=%d" % [
		body_id, hinge_id, motor_id, sensor_id])

	# Apply a constant motor torque and let gravity act; integrate over time.
	world.set_ctrl(motor_id, 0.15)

	var start_angle: float = world.get_qpos()[0]
	print("--- stepping simulation (dt=%.3fs) ---" % world.get_timestep())
	for i in range(6):
		world.step(50) # 50 * dt of simulated time
		var qpos := world.get_qpos()
		var quat := world.body_world_quaternion(body_id)
		var sensor := world.get_sensor(sensor_id)
		print("t=%.2fs  hinge=%+.4f rad  quat=(%.3f,%.3f,%.3f,%.3f)  sensor[hinge_pos]=%+.4f" % [
			world.get_time(), qpos[0], quat.x, quat.y, quat.z, quat.w,
			(sensor[0] if sensor.size() > 0 else 0.0)])

	var end_angle: float = world.get_qpos()[0]
	var moved: bool = abs(end_angle - start_angle) > 0.01

	# mj_step evaluates sensors before integrating, so sensordata lags qpos by
	# one step; forward() recomputes derived quantities for the current state.
	world.forward()
	var sensordata := world.get_sensordata()
	var sensor_matches_qpos: bool = sensordata.size() == 2 and abs(sensordata[0] - end_angle) < 1e-6
	print("start_angle=%.4f  end_angle=%.4f  moved=%s  sensordata=%s  sensor_matches_qpos=%s" % [
		start_angle, end_angle, str(moved), str(sensordata), str(sensor_matches_qpos)])

	# Debug snapshot: contacts, energy and warnings.
	print("debug: ncon=%d  KE=%.4f  PE=%.4f  warnings=%s" % [
		world.get_ncon(), world.get_kinetic_energy(),
		world.get_potential_energy(), str(world.get_warnings())])
	print("debug_info=%s" % str(world.get_debug_info()))

	# Export-safe string load (uses MuJoCo's VFS; no filesystem path required).
	var w2 := MjWorld.new()
	add_child(w2)
	var strload_ok := w2.load_model_from_string('<mujoco><worldbody><body name="b"><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>')
	strload_ok = strload_ok and w2.get_nbody() == 2

	# Multi-file MJCF: <include> + a mesh asset in a subdirectory, resolved via VFS.
	var w3 := MjWorld.new()
	add_child(w3)
	var multifile_ok := w3.load_model("res://models/composite/scene.xml") and w3.get_nbody() == 2
	print("multifile_ok=%s (err='%s')" % [str(multifile_ok), w3.get_last_error()])

	# Loud error handling: a wrong-sized set_qpos must be rejected (returns false).
	var bad_rejected: bool = not world.set_qpos(PackedFloat64Array([1.0, 2.0, 3.0]))
	# UTF-8-safe name round-trip.
	var name_roundtrip: bool = world.body_name(world.body_id("pendulum")) == "pendulum"
	# step(0) must be a no-op.
	var t_before := world.get_time()
	world.step(0)
	var step0_noop: bool = world.get_time() == t_before
	print("checks: strload=%s bad_set_rejected=%s name_roundtrip=%s step0_noop=%s" % [
		str(strload_ok), str(bad_rejected), str(name_roundtrip), str(step0_noop)])

	var passed: bool = moved and body_id >= 0 and world.get_nq() == 1 and world.get_nu() == 1 \
		and world.get_nsensor() == 2 and world.get_njnt() == 1 and sensor_matches_qpos \
		and strload_ok and multifile_ok and bad_rejected and name_roundtrip and step0_noop
	if passed:
		print("SMOKE TEST: PASS")
		get_tree().quit(0)
	else:
		print("SMOKE TEST: FAIL")
		get_tree().quit(1)
