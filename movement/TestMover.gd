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

onready var phys_fps = ProjectSettings.get_setting("physics/common/physics_fps")

var slip_sphere:SphereShape

# -------------------------------------------------------------Movement Settings
var jump_height := 2.5
var jump_duration := 0.4

var jump_grace_ticks := 10
var jump_try_ticks := 4

var ticks_until_in_air := 5

var gravity := (2.0 * jump_height) / (pow(jump_duration, 2))
var jump_force := gravity * jump_duration
var speed := 8
var speed_limit := 35.5
var h_speed_limit_sqr := pow(speed_limit, 2)
var speed_zero_limit := 0.0005 # if speed^2 falls below this, set it to 0
var acceleration := 12.0
var acceleration_in_air := 2.0

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
var floor_limit = 0.6
var last_gravity_applied = Vector3()
var n_delta_lim = 0.000001
var ramp_transition = false
func calculate_movement(delta:float):
	"""
		Todos
		- Sticky edges
		- Ramp bump
	"""
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
	
	var old_floor_normal = floor_normal

	
	# get_dist_to_ground() < is_grounded_threshold
	if is_on_floor():
#		print("floored")
		floor_normal = get_floor_normal()
#		velocity -= speed * floor_normal
#		velocity = velocity.slide(floor_normal)
		last_gravity_applied = 2 * floor_normal
#		if floor_normal.dot(Vector3.UP) > floor_limit:
##			print("flat")
#			last_gravity_applied = speed * floor_normal
#			velocity -= last_gravity_applied
#		else:
##			print("corner")
#			last_gravity_applied = 0.1 * floor_normal
#			velocity -= last_gravity_applied
		ticks_since_on_floor = 0
		if did_n_change(old_floor_normal, floor_normal):
			if (old_floor_normal.dot(Vector3.UP) >= cos(PI / 4) and
			velocity.dot(old_floor_normal) < velocity.dot(floor_normal)): # going over edge / hump
				var vel_mag = velocity.length_squared()
				if old_floor_normal.dot(Vector3.UP) < floor_normal.dot(Vector3.UP):
#					print("flattening", Network.physics_tick_id)
					velocity = velocity.slide(floor_normal)
#					velocity *= sqrt(vel_mag / velocity.length_squared())
				else:
#					velocity = velocity.linear_interpolate(velocity.slide(floor_normal), 1.0)
#					velocity *= sqrt(vel_mag / velocity.length_squared())
#					print("sharpening", Network.physics_tick_id)
					pass
#				velocity = velocity.slide(floor_normal) - 4 * floor_normal
				
				pass
#			print("floor normal changed @fr ", Network.physics_tick_id)
	else:
#		print("air")
		if ticks_since_on_floor == 0:
			print(velocity.y - velocity.slide(floor_normal).y)
			velocity.y = 0.8 * velocity.slide(floor_normal).y
#			velocity += last_gravity_applied.slide(floor_normal)
#			var h_vel = Vector2(velocity.x, velocity.z)
#			var h_norm = Vector2(floor_normal.x, floor_normal.z)
#			print(h_vel.dot(h_norm))
#			if h_vel.dot(h_norm) > 1:
#				var fall_compensation = last_gravity_applied.y * Vector3.UP
#				print(fall_compensation)
#				velocity += fall_compensation
#			velocity = 0.5 * velocity.linear_interpolate(velocity.slide(floor_normal), 1)
#			if floor_normal.dot(Vector3.UP) > floor_limit:
#				velocity += speed * floor_normal
#				velocity = velocity.linear_interpolate(velocity.slide(floor_normal), 1)
#			else:
#				velocity += speed * floor_normal
#				velocity = velocity.linear_interpolate(velocity.slide(floor_normal), 1)
		floor_normal = Vector3.UP
		last_gravity_applied = gravity * delta * floor_normal
		ticks_since_on_floor += 1
	velocity -= last_gravity_applied
#	print(velocity.y)
#	print(is_on_floor())
#	print(velocity)
	
	
#	print(get_floor_normal().dot(Vector3.UP))
#	print(is_pushing_wall())
#	print(is_on_wall())
#	print(is_on_ceiling())
#	print(get_slide_count())
	
	if (move_slice[MOVE.JUMP]): 
#	and ticks_since_on_floor < jump_grace_ticks and
#	ticks_since_last_jump > jump_grace_ticks):
		velocity += last_gravity_applied
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

func did_n_change(n_old:Vector3, n_new:Vector3):
	return n_old.distance_squared_to(n_new) > n_delta_lim
#	(not n_old.is_equal_approx(n_new)) and (not n_old.is_equal_approx(Vector3.UP))
#	n_old.distance_squared_to(n_new) > n_delta_lim

var h_vel := Vector3()
var g_vel := Vector3()
var g_grnd := 10.0
var g_edge := 0.1

func calculate_movement_2(delta:float):
#	h_vel = real_vel
#	h_vel.y = 0
	var target_hvel = speed * (
			move_slice[MOVE.X_DIR] * transform.basis.x +
			move_slice[MOVE.Z_DIR] * -transform.basis.z).normalized()
	target_hvel.y = 0
	
	if ticks_since_on_floor > ticks_until_in_air:
		h_vel = h_vel.linear_interpolate(
			target_hvel, acceleration_in_air * delta)
	else:
		h_vel = h_vel.linear_interpolate(
			target_hvel, acceleration * delta)
	
	# get_dist_to_ground() < is_grounded_threshold
	if is_on_floor():
		print("floored")
		floor_normal = get_floor_normal()
		floor_snap = floor_normal
#		velocity -= speed * floor_normal
		if floor_normal.dot(Vector3.UP) > floor_limit:
#			print("flat")
			g_vel = g_vel.linear_interpolate(
				-g_grnd * floor_normal, acceleration * delta)
		else:
#			print("edge")
			g_vel = g_vel.linear_interpolate(
				-g_edge * floor_normal, acceleration * delta)
		ticks_since_on_floor = 0
	else:
		print("air")
		floor_normal = Vector3.UP
		
		if ticks_since_on_floor == 0:
			g_vel = -10 *gravity * delta * floor_normal
		else:
			g_vel -= gravity * delta * floor_normal
		
#		g_vel -= gravity * delta * floor_normal
		
		ticks_since_on_floor += 1
	
	if is_on_ceiling():
		g_vel = Vector3()
	
#	print(velocity.y)
#	print(is_on_floor())
#	print(velocity)
#	print(get_floor_normal())
#	print(is_on_wall())
	
	if (move_slice[MOVE.JUMP]): 
#	and ticks_since_on_floor < jump_grace_ticks and
#	ticks_since_last_jump > jump_grace_ticks):
		g_vel.y = jump_force
#		gravity_component = jump_force * Vector3.UP
		ticks_since_on_floor = jump_grace_ticks
		ticks_since_last_jump = 0
	
	ticks_since_last_jump += 1
	
	velocity = h_vel + g_vel

func calculate_movement_3(delta:float):
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
	
	var old_floor_normal = floor_normal
	
	if is_on_floor():
#		print("floored")
		floor_normal = get_floor_normal()
		floor_snap = floor_normal
		last_gravity_applied = floor_normal
		velocity -= last_gravity_applied
		ticks_since_on_floor = 0
	else:
		floor_normal = Vector3.UP
		floor_snap = floor_normal
		velocity -= gravity * delta * floor_normal
		ticks_since_on_floor += 1
	
#	if did_n_change(old_floor_normal, floor_normal):
#		if (old_floor_normal.dot(Vector3.UP) > 0.8 and
#		velocity.dot(old_floor_normal) < velocity.dot(floor_normal)):
#			print("compensating @tick ", Network.physics_tick_id)
#			velocity = velocity.slide(floor_normal)
#			velocity += 0.8 * last_gravity_applied
#		print("floor normal changed @fr ", Network.physics_tick_id)
	
#	print(velocity.y)
#	print(is_on_floor())
#	print(velocity)
#	print(get_floor_normal().y)
#	print(is_pushing_wall())
#	print(is_on_wall())
#	print(get_slide_count())
	
	if (move_slice[MOVE.JUMP]): 
		velocity.y = jump_force
		ticks_since_on_floor = jump_grace_ticks
		ticks_since_last_jump = 0
	
	ticks_since_last_jump += 1

func calculate_movement_4(delta:float):
	pre_move_origin = transform.origin
	var target_velocity =  (
		move_slice[MOVE.X_DIR] * transform.basis.x + 
		move_slice[MOVE.Z_DIR] * -transform.basis.z).normalized()
	
	var old_floor_normal = floor_normal
	
	if is_on_floor():
#		print("floored")
		floor_normal = get_floor_normal()
		last_gravity_applied = floor_normal
		target_velocity = target_velocity * speed + velocity.y * Vector3.UP
		velocity = velocity.linear_interpolate(
			target_velocity, acceleration * delta)
		
		use_cl_norm = Vector3()
		ticks_since_on_floor = 0
	else:
#		print("air")
		floor_normal = Vector3.UP
		last_gravity_applied = gravity * delta * floor_normal
		target_velocity = (target_velocity.slide(floor_normal).normalized() * speed)
		velocity = velocity.linear_interpolate(
			target_velocity, acceleration_in_air * delta)
		velocity -= last_gravity_applied
		ticks_since_on_floor += 1
		
	
#	if ticks_since_on_floor > ticks_until_in_air:
#		velocity = velocity.linear_interpolate(
#			target_velocity, acceleration_in_air * delta)
#		velocity -= last_gravity_applied
#	else:
#		velocity = velocity.linear_interpolate(
#			target_velocity, acceleration * delta)
#		velocity -= last_gravity_applied
		
#	print(floor_normal)
	
	
	if did_n_change(old_floor_normal, floor_normal):
		if (old_floor_normal.dot(Vector3.UP) > 0.8 and
		velocity.dot(old_floor_normal) < velocity.dot(floor_normal)):
#			print("compensating @tick ", Network.physics_tick_id)
#			velocity = velocity.slide(floor_normal)
			velocity += 0.75 * last_gravity_applied
#		print("floor normal changed @fr ", Network.physics_tick_id)
	
#	print(velocity)
	
	if (move_slice[MOVE.JUMP]): 
		velocity.y = jump_force
		ticks_since_on_floor = jump_grace_ticks
		ticks_since_last_jump = 0
	
#	print(velocity.y)
	
	ticks_since_last_jump += 1
	
#	process_slides()

var n_grnd_avg = Vector3()
func process_slides():
	n_grnd_avg = Vector3()
	for idx in range(get_slide_count()):
		var c_norm := get_slide_collision(idx).normal
		print(c_norm)
		if c_norm.dot(Vector3.UP) > cos(floor_limit):
			n_grnd_avg += c_norm
	
	n_grnd_avg = n_grnd_avg.normalized()
	
#	print(n_grnd_avg)

var inability_to_handle_sigma_male = 2.0

var real_vel = Vector3()
var use_cl_norm = Vector3()
func apply_movement():
#	print("pre-move vel: ", velocity)
	var pre_vel = velocity
	if not velocity.is_equal_approx(Vector3()):
		var slid_vel = move_and_slide(velocity, Vector3.UP, false, 4)
#		var slid_vel = move_and_slide_with_snap(
#			velocity, 
#			-floor_normal if ticks_since_last_jump < 0 else Vector3(), 
#			Vector3.UP, false, 4)
		velocity = slid_vel
#		print("moved")
		real_vel = phys_fps * (transform.origin - pre_move_origin)
		real_vel.y = 0.5 * (real_vel.y + velocity.y)
#		print(floor_normal.dot(Vector3.UP))
		if (not velocity.is_equal_approx(real_vel)):
#			print("VEL: ", velocity, " RVEL: ", real_vel, " D: ", velocity.distance_squared_to(real_vel))
#			print("bumping ", Network.physics_tick_id)
#			print(real_vel)
			velocity = velocity.linear_interpolate(real_vel, 0.75)
#		pre_move_origin = transform.origin
#		print("pos-move vel: ", velocity)
#	else:
#		velocity = Vector3()

func apply_movement_2():
	if not velocity.is_equal_approx(Vector3()):
		velocity = move_and_slide(velocity, Vector3.UP, false, 4)

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

func is_pushing_wall():
	var space_state = get_world().direct_space_state
	var hvel = velocity
	hvel.y = 0
	hvel = hvel.normalized() * 1.1
	var result:Dictionary = space_state.intersect_ray(
		transform.origin, 
		transform.origin + hvel,
		[self])
	
#	print(hvel)
	
#	if result:
#		print(result.position)
	
	return not result.empty()

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
