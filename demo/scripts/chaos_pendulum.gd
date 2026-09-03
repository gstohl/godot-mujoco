extends Node3D

# Visual chaotic double pendulum. MuJoCo integrates the passive dynamics; the
# two joint angles drive nested Godot pivots (the child pivot inherits the
# parent's rotation, matching MuJoCo's relative elbow joint).

var world: MjWorld

@onready var upper_pivot: Node3D = $Pivot/UpperPivot
@onready var lower_pivot: Node3D = $Pivot/UpperPivot/LowerPivot
@onready var status_label: Label = $UI/Status

func _ready() -> void:
	world = MjWorld.new()
	add_child(world)
	if not world.load_model("res://models/double_pendulum.xml"):
		push_error("Failed to load MuJoCo model: " + world.get_last_error())
		return
	# Start with the upper link near-inverted — a chaotic regime.
	world.set_qpos(PackedFloat64Array([3.1, 0.2]))
	world.forward()
	print("MuJoCo ", world.get_mujoco_version(), " chaotic double pendulum ready")
	if status_label:
		status_label.text = "MuJoCo %s — chaotic double pendulum (in-engine)" % world.get_mujoco_version()

func _physics_process(_delta: float) -> void:
	if world == null or not world.is_ready():
		return
	world.step(4) # 4 * 0.005s per 1/60s frame
	var q := world.get_qpos()
	# Hinge axis is MuJoCo +Y (maps to Godot -Z); shoulder is absolute, elbow is
	# relative — nesting composes them for us.
	upper_pivot.rotation = Vector3(0.0, 0.0, -float(q[0]))
	lower_pivot.rotation = Vector3(0.0, 0.0, -float(q[1]))
