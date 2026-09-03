extends Node

# Headless end-to-end proof that MuJoCo runs fully inside Godot through the
# GDExtension, with no bridge library and no external MuJoCo install.
# Run with:
#   godot --headless --path gdextension/demo res://HeadlessTest.tscn

func _ready() -> void:
	var world := MjWorld.new()
	add_child(world)

	print("MuJoCo version reported by extension: ", world.get_mujoco_version())

	if not world.load_model("res://models/pendulum.xml"):
		push_error("load_model failed: " + world.get_last_error())
		get_tree().quit(1)
		return

	print("Model loaded. nq=%d nv=%d nu=%d nbody=%d" % [
		world.get_nq(), world.get_nv(), world.get_nu(), world.get_nbody()])

	var body_id := world.body_id("pendulum")
	var hinge_id := world.joint_id("hinge")
	var motor_id := world.actuator_id("hinge_motor")
	print("resolved ids: body=%d hinge=%d motor=%d" % [body_id, hinge_id, motor_id])

	# Apply a constant motor torque and let gravity act; integrate over time.
	world.set_ctrl(motor_id, 0.15)

	var start_angle: float = world.get_qpos()[0]
	print("--- stepping simulation (dt=0.01s) ---")
	for i in range(6):
		world.step(50) # 50 * 0.01s = 0.5 s of simulated time
		var qpos := world.get_qpos()
		var pos := world.body_world_position(body_id)
		print("t=%.2fs  hinge=%+.4f rad  body_pos=(%.3f, %.3f, %.3f)" % [
			(i + 1) * 0.5, qpos[0], pos.x, pos.y, pos.z])

	var end_angle: float = world.get_qpos()[0]
	var moved: bool = abs(end_angle - start_angle) > 0.01

	print("start_angle=%.4f  end_angle=%.4f  moved=%s" % [start_angle, end_angle, str(moved)])

	var passed: bool = moved and body_id >= 0 and world.get_nq() == 1 and world.get_nu() == 1
	if passed:
		print("SMOKE TEST: PASS")
		get_tree().quit(0)
	else:
		print("SMOKE TEST: FAIL")
		get_tree().quit(1)
