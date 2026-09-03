extends Node

# Headless test of a chaotic double pendulum running fully in-engine via MuJoCo.
# Demonstrates the hallmark of chaos: sensitive dependence on initial
# conditions. Two independent MjWorld instances start from *nearly* identical
# states (a 1e-5 rad perturbation) and their trajectories diverge to O(1).
# Also exercises multi-instance isolation (two separate mjModel/mjData).

func _ready() -> void:
	var a := MjWorld.new()
	var b := MjWorld.new()
	add_child(a)
	add_child(b)

	for w in [a, b]:
		if not w.load_model("res://models/double_pendulum.xml"):
			push_error("load_model failed: " + w.get_last_error())
			get_tree().quit(1)
			return

	print("MuJoCo %s double pendulum: nq=%d nv=%d nbody=%d njnt=%d" % [
		a.get_mujoco_version(), a.get_nq(), a.get_nv(), a.get_nbody(), a.get_njnt()])

	# Identical start (upper near-inverted — a chaotic regime), except a tiny
	# perturbation on B's shoulder angle.
	var eps := 0.00001 # 1e-5 rad
	a.set_qpos(PackedFloat64Array([3.1, 0.2]))
	b.set_qpos(PackedFloat64Array([3.1 + eps, 0.2]))
	a.forward()
	b.forward()
	print("initial shoulder perturbation = %.5f rad" % eps)

	var e0: float = a.get_kinetic_energy() + a.get_potential_energy()
	print("initial energy: KE=%.4f  PE=%.4f  total=%.4f J" % [
		a.get_kinetic_energy(), a.get_potential_energy(), e0])

	var dt := a.get_timestep()
	var diverged_time := -1.0
	var max_div := 0.0
	print("--- stepping two near-identical double pendulums (dt=%.3fs) ---" % dt)
	for i in range(1, 1201): # 1200 * 0.005 s = 6 s
		a.step(1)
		b.step(1)
		var da: float = abs(a.get_qpos()[0] - b.get_qpos()[0])
		max_div = max(max_div, da)
		if diverged_time < 0.0 and da > 0.5:
			diverged_time = i * dt
		if i % 200 == 0:
			var qa := a.get_qpos()
			var qb := b.get_qpos()
			print("t=%.2fs  |Δshoulder|=%.4f rad  |Δelbow|=%.4f rad" % [
				i * dt, abs(qa[0] - qb[0]), abs(qa[1] - qb[1])])

	print("max |Δshoulder| over 6s = %.4f rad; first exceeded 0.5 rad at t=%.2fs" % [
		max_div, diverged_time])

	# Debug: a passive system should approximately conserve energy.
	var e1: float = a.get_kinetic_energy() + a.get_potential_energy()
	print("debug: final energy total=%.4f J (drift %.2f%%), ncon=%d, warnings=%s" % [
		e1, 100.0 * (e1 - e0) / e0 if e0 != 0.0 else 0.0, a.get_ncon(), str(a.get_warnings())])

	# A 1e-5 rad perturbation growing past 0.5 rad is the chaotic signature.
	var passed: bool = a.get_nq() == 2 and a.get_nbody() == 3 \
		and diverged_time > 0.0 and max_div > 1.0
	if passed:
		print("CHAOS TEST: PASS")
		get_tree().quit(0)
	else:
		print("CHAOS TEST: FAIL")
		get_tree().quit(1)
