extends KinematicBody

class_name TestMover

enum MOVE { # move instruc
	PROCESSED, # has been processed
	JUMP,
	X_DIR,
	Z_DIR,
	LOOK, # yaw and pitch
	LOOK_DELTA} # did look change since last frame	

export(NodePath) var visual_root_path:NodePath
export(NodePath) var camera_path:NodePath
export(NodePath) var move_collider_path:NodePath
export(NodePath) var hurt_collider_path:NodePath
onready var visual_root:Spatial = get_node(visual_root_path)
onready var camera:Camera = get_node(camera_path)
onready var move_collider:CollisionShape = get_node(move_collider_path)
onready var hurt_collider:CollisionShape = get_node(hurt_collider_path)

var slip_sphere:SphereShape

# -------------------------------------------------------------Movement Settings
var jump_force := 15.0
var jump_grace_ticks := 10
var jump_try_ticks := 4

var ticks_until_in_air := 5

var gravity := 45.0
var speed := 8
var speed_limit := 35.5
var h_speed_limit_sqr := pow(speed_limit, 2)
var speed_zero_limit := 0.0005 # if speed^2 falls below this, set it to 0
var acceleration := 11.0
var acceleration_in_air := 3.0

var is_grounded_threshold := 0.04
var ground_snap_threshold := 0.0
var slip_radius := 1.0
var character_feet_offset := 1.0 # how far below character origin is its feet?

# -----------------------------------------------------------------Movement Vars
var yaw := 0.0
var pitch := 0.0
var ticks_since_last_jump := jump_grace_ticks
var ticks_since_on_floor := 0
var ticks_since_on_wall := 0
var velocity := Vector3()
var is_grounded_query:PhysicsShapeQueryParameters
var move_slice:Array
var gravity_component := Vector3()

# ------------------------------------------------------------------Network vars
var move_buffer:PoolBuffer

var last_frame_yaw := 0.0
var avg_yaw_delta := 0.0

func _ready():
	init_grounded_query()

	init_move_recording()

func init_grounded_query():
	slip_sphere = SphereShape.new()
	slip_sphere.radius = slip_radius
	
	is_grounded_query = PhysicsShapeQueryParameters.new()
	is_grounded_query.exclude = [
		self, 
		move_collider, 
		hurt_collider]
#	is_grounded_query.margin = 0.1
	is_grounded_query.set_shape(slip_sphere)

func init_move_recording():
	move_slice = []
	move_slice.resize(MOVE.size())
	move_slice[MOVE.PROCESSED] = 0
	move_slice[MOVE.JUMP] = 0
	move_slice[MOVE.X_DIR] = 0
	move_slice[MOVE.Z_DIR] = 0
	move_slice[MOVE.LOOK] = Vector2(0.0, 0.0)
	move_slice[MOVE.LOOK_DELTA] = 0

	var move_stubs = []
	move_stubs.resize(MOVE.size())
	move_stubs[MOVE.PROCESSED] = PoolByteArray()
	move_stubs[MOVE.JUMP] = PoolByteArray()
	move_stubs[MOVE.X_DIR] = PoolByteArray()
	move_stubs[MOVE.Z_DIR] = PoolByteArray()
	move_stubs[MOVE.LOOK] = PoolVector2Array()
	move_stubs[MOVE.LOOK_DELTA] = PoolByteArray()
	move_buffer = PoolBuffer.new(move_stubs)

func get_dist_to_ground():
	"""
		Custom implementation of is_on_floor()
		Not needed when using bulletphysics
	"""
	var space_state = get_world().direct_space_state
	is_grounded_query.transform = get_global_transform()
	is_grounded_query.transform.origin += ( # make slip sphere touch ground
		Vector3.DOWN * (character_feet_offset - slip_radius))
	var cast_result = space_state.cast_motion(is_grounded_query, Vector3.DOWN)
	return cast_result[0]

var up_dir = Vector3.UP
var floor_normal = Vector3.UP
var fnh_lerp = 0.5
var floor_snap = Vector3()
var pre_move_origin = Vector3()
var floor_limit = 0.4
func calculate_movement(delta:float):
	pre_move_origin = transform.origin
	var target_velocity = (
		speed * (
			move_slice[MOVE.X_DIR] * transform.basis.x +
			move_slice[MOVE.Z_DIR] * -transform.basis.z).normalized()
		+ velocity.y * Vector3.UP)
	
	if ticks_since_on_floor > ticks_until_in_air:
		velocity = velocity.linear_interpolate(
			target_velocity, acceleration_in_air * delta)
	else:
		velocity = velocity.linear_interpolate(
			target_velocity, acceleration * delta)
	
	# get_dist_to_ground() < is_grounded_threshold
	if is_on_floor():
#		print("floored")
		floor_normal = get_floor_normal()
		floor_snap = floor_normal
		if floor_normal.dot(Vector3.UP) < floor_limit:
			velocity -= 10 * floor_normal
		else:
			velocity -= 4 * floor_normal
		ticks_since_on_floor = 0
	else:
#		print("air")
		if ticks_since_on_floor == 0:
			if floor_normal.dot(Vector3.UP) < floor_limit:
				velocity += 10 * floor_normal
			else:
				velocity += 4 * floor_normal
		floor_normal = Vector3.UP
		floor_snap = floor_normal
		velocity -= gravity * delta * floor_normal
		ticks_since_on_floor += 1
	
#	print(is_on_floor())
#	print(velocity)
#	print(get_floor_normal())
#	print(is_on_wall())
	
	if (move_slice[MOVE.JUMP]): 
#	and ticks_since_on_floor < jump_grace_ticks and
#	ticks_since_last_jump > jump_grace_ticks):
		velocity.y = jump_force
#		gravity_component = jump_force * Vector3.UP
		ticks_since_on_floor = jump_grace_ticks
		ticks_since_last_jump = 0
	
#	process_slides()
	
	ticks_since_last_jump += 1
	
	"""
		var vel_h_mag_sqr = pow(velocity.x, 2) + pow(velocity.z, 2)
		if vel_h_mag_sqr > h_speed_limit_sqr:
			var h_scale_fac = sqrt(h_speed_limit_sqr / vel_h_mag_sqr)
			velocity.x *= h_scale_fac
			velocity.z *= h_scale_fac

		var vel_mag_sqr = vel_h_mag_sqr + pow(velocity.y, 2)
		if vel_mag_sqr < speed_zero_limit and ticks_since_on_floor == 0:
			velocity = Vector3()

		velocity += floor_snap

		var sigma_offset = transform.origin - Network.sigma_position
		if sigma_offset.length_squared() < 5.0:
			velocity += (
				sigma_offset.normalized() * 
				(inability_to_handle_sigma_male / sigma_offset.length_squared()))
	"""

func process_slides():
	var on_wall := false
	for idx in range(get_slide_count()):
		var slide_collision := get_slide_collision(idx)
		on_wall = on_wall || slide_collision.normal.dot(Vector3.UP) < 0.7
	print(on_wall)

var inability_to_handle_sigma_male = 2.0

func apply_movement():
#	print("pre-move vel: ", velocity)
	if not velocity.is_equal_approx(Vector3()):
		var slid_vel = move_and_slide(velocity, Vector3.UP, false, 4, 0.9)
#		var slid_vel = move_and_slide_with_snap(velocity, floor_snap, Vector3.UP, false, 3)
		velocity = slid_vel
#		print("moved")
		var real_vel = 90 * (transform.origin - pre_move_origin)
		real_vel.y = velocity.y
#		print(floor_normal.dot(Vector3.UP))
		if velocity.distance_squared_to(real_vel) > 0.1:
			velocity = real_vel
		pre_move_origin = transform.origin
#		print("pos-move vel: ", velocity)

func apply_movement_custom(delta):
	var result := PhysicsTestMotionResult.new()
	
	PhysicsServer.body_test_motion(
		get_rid(), transform, velocity * delta, true, result)
	transform.origin += result.motion
	if result.collision_normal:
		velocity = result.motion_remainder.slide(result.collision_normal)
	
	PhysicsServer.body_test_motion(
		get_rid(), transform, velocity * delta, true, result)
		
	transform.origin += result.motion
	if result.collision_normal:
		velocity = result.motion_remainder.slide(result.collision_normal)

func show_angle_change():
	avg_yaw_delta = (
		0.9 * avg_yaw_delta + 
		0.1 * shortest_deg_between(yaw, last_frame_yaw))
	print(shortest_deg_between(yaw, last_frame_yaw))
	last_frame_yaw = yaw

static func deg_to_deg360(deg : float):
	deg = fmod(deg, 360.0)
	if deg < 0.0:
		deg += 360.0
	return deg

func shortest_deg_between(deg1 : float, deg2 : float):
	return min(
		abs(deg1 - deg2),
		min(abs((deg1 - 360.0) - deg2), abs((deg2 - 360.0) - deg1)))
