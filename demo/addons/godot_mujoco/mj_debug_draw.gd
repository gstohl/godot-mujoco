class_name MjDebugDraw
extends Node3D

# Visual debug overlay for an MjWorld: draws MuJoCo's debug geometry (contact
# points, contact-force arrows, body frames, joint axes, center of mass) as
# line primitives on top of the scene. Assign `world` (in the inspector or from
# code) and toggle what to show. MuJoCo is Z-up; this maps its world frame into
# Godot's Y-up.

@export var world: MjWorld
@export var show_body_frames := true
@export var show_joints := true
@export var show_com := true
@export var show_contacts := true
@export var show_contact_forces := true
@export var axis_length := 0.2
@export var force_scale := 0.008

var _im := ImmediateMesh.new()
var _mi := MeshInstance3D.new()
var _mat := StandardMaterial3D.new()
var _begun := false

func _ready() -> void:
	_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_mat.vertex_color_use_as_albedo = true
	_mat.no_depth_test = true # draw the debug overlay on top of geometry
	_mi.mesh = _im
	_mi.material_override = _mat
	add_child(_mi)

# MuJoCo (Z-up, right-handed) -> Godot (Y-up). A proper rotation (-90° about X).
func m2g(v: Vector3) -> Vector3:
	return Vector3(v.x, v.z, -v.y)

func _process(_delta: float) -> void:
	_im.clear_surfaces()
	_begun = false
	if not is_instance_valid(world) or not world.is_ready():
		return

	if show_body_frames:
		for i in range(1, world.get_nbody()):
			var t := world.body_world_transform(i)
			var o := m2g(t.origin)
			_line(o, o + m2g(t.basis.x) * axis_length, Color.RED)
			_line(o, o + m2g(t.basis.y) * axis_length, Color.GREEN)
			_line(o, o + m2g(t.basis.z) * axis_length, Color.BLUE)

	if show_joints:
		for j in range(world.get_njnt()):
			var a := m2g(world.get_joint_anchor(j))
			var ax := m2g(world.get_joint_axis(j))
			_line(a - ax * axis_length, a + ax * axis_length, Color.YELLOW)

	if show_com:
		_cross(m2g(world.get_center_of_mass()), 0.08, Color.MAGENTA)

	if show_contacts:
		for c in world.get_contacts():
			var p: Vector3 = m2g(c["pos"])
			_cross(p, 0.05, Color.AQUA)
			if show_contact_forces:
				var f: Vector3 = m2g(c["force"]) * force_scale
				_line(p, p + f, Color.ORANGE)

	# ImmediateMesh errors if surface_end() is called with no vertices, so only
	# finish the surface when at least one line was drawn.
	if _begun:
		_im.surface_end()

func _ensure_begun() -> void:
	if not _begun:
		_im.surface_begin(Mesh.PRIMITIVE_LINES)
		_begun = true

func _line(a: Vector3, b: Vector3, col: Color) -> void:
	_ensure_begun()
	_im.surface_set_color(col)
	_im.surface_add_vertex(a)
	_im.surface_set_color(col)
	_im.surface_add_vertex(b)

func _cross(p: Vector3, s: float, col: Color) -> void:
	_line(p - Vector3(s, 0, 0), p + Vector3(s, 0, 0), col)
	_line(p - Vector3(0, s, 0), p + Vector3(0, s, 0), col)
	_line(p - Vector3(0, 0, s), p + Vector3(0, 0, s), col)
