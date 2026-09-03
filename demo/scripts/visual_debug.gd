extends Node3D

# Visual-debug showcase: balls fall and collide on a floor while an MjDebugDraw
# overlay renders MuJoCo's contact points, contact-force arrows, body frames and
# center of mass. Ball meshes are placed directly from MuJoCo body transforms.

const BALL_RADIUS := 0.15

var world: MjWorld
var ball_nodes: Array = []

@onready var debug_draw: MjDebugDraw = $MjDebugDraw
@onready var status_label: Label = $UI/Status

func _ready() -> void:
	world = MjWorld.new()
	add_child(world)
	if not world.load_model("res://models/contacts.xml"):
		push_error("Failed to load model: " + world.get_last_error())
		return
	debug_draw.world = world

	# One Godot sphere per MuJoCo ball body (body 0 is the world).
	for i in range(1, world.get_nbody()):
		var mi := MeshInstance3D.new()
		var sm := SphereMesh.new()
		sm.radius = BALL_RADIUS
		sm.height = BALL_RADIUS * 2.0
		mi.mesh = sm
		var mat := StandardMaterial3D.new()
		mat.albedo_color = Color.from_hsv(float(i) / float(world.get_nbody()), 0.55, 0.9)
		mi.material_override = mat
		add_child(mi)
		ball_nodes.append(mi)

	status_label.text = "MuJoCo %s — visual debug (contacts, forces, frames, COM)" % world.get_mujoco_version()

func _physics_process(_delta: float) -> void:
	if world == null or not world.is_ready():
		return
	world.step(2)
	for idx in range(ball_nodes.size()):
		ball_nodes[idx].position = _m2g(world.body_world_position(idx + 1))

func _m2g(v: Vector3) -> Vector3:
	return Vector3(v.x, v.z, -v.y)
