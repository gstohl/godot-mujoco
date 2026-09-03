extends Node

# Headless validation of the visual-debug data getters: after the balls settle
# on the floor there must be active contacts, and their forces must point up
# (supporting the weight). This regression-protects the data that MjDebugDraw
# renders.

func _ready() -> void:
	var w := MjWorld.new()
	add_child(w)
	if not w.load_model("res://models/contacts.xml"):
		push_error("load_model failed: " + w.get_last_error())
		get_tree().quit(1)
		return

	print("MuJoCo %s contacts model: nbody=%d" % [w.get_mujoco_version(), w.get_nbody()])

	for _i in range(600): # 3 s to fall and settle
		w.step(1)

	var contacts := w.get_contacts()
	var ncon := w.get_ncon()
	print("after settle: ncon=%d, contacts_returned=%d" % [ncon, contacts.size()])

	var total_up := 0.0
	for c in contacts:
		total_up += (c["force"] as Vector3).z # MuJoCo is Z-up: support is +z
	print("sum contact force z = %.2f N; COM=%s" % [total_up, str(w.get_center_of_mass())])
	if contacts.size() > 0:
		print("example contact: %s" % str(contacts[0]))

	var passed: bool = ncon > 0 and contacts.size() == ncon and total_up > 0.0
	if passed:
		print("VISUAL DEBUG DATA TEST: PASS")
		get_tree().quit(0)
	else:
		print("VISUAL DEBUG DATA TEST: FAIL")
		get_tree().quit(1)
