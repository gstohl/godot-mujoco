extends Node3D

# Visual demo: a MuJoCo-simulated hinge pendulum driven every physics tick,
# with the joint state mapped onto a Godot mesh. MuJoCo runs entirely inside
# the engine via the GDExtension.

var world: MjWorld
var body_id: int = -1
var motor_id: int = -1

@onready var pendulum: Node3D = $Pendulum
@onready var status_label: Label = $UI/Status

func _ready() -> void:
	world = MjWorld.new()
	add_child(world)

	if not world.load_model("res://models/pendulum.xml"):
		push_error("Failed to load MuJoCo model: " + world.get_last_error())
		if status_label:
			status_label.text = "MuJoCo load FAILED"
		return

	body_id = world.body_id("pendulum")
	motor_id = world.actuator_id("hinge_motor")
	print("MuJoCo ", world.get_mujoco_version(), " ready — nq=", world.get_nq(),
		" nu=", world.get_nu())
	if status_label:
		status_label.text = "MuJoCo %s running in Godot (no bridge)" % world.get_mujoco_version()

func _physics_process(_delta: float) -> void:
	if world == null or not world.is_ready():
		return

	# Gentle oscillating torque so the pendulum keeps swinging on screen.
	var t := Time.get_ticks_msec() / 1000.0
	world.set_ctrl(motor_id, 0.25 * sin(t * 2.0))
	world.step(2)

	# MuJoCo hinge angle drives the mesh rotation. The hinge axis is MuJoCo +Y,
	# which maps to Godot -Z, so negate the angle for a faithful direction.
	var angle: float = world.get_qpos()[0]
	pendulum.rotation = Vector3(0.0, 0.0, -angle)
