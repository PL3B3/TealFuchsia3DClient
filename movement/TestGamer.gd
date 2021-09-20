extends TestMover

class_name TestGamer

onready var target = preload("res://common/game/Target.tscn")

enum STATE {
	POSITION,
	VELOCITY, # slid vel at end of previous frame
	HEALTH,
	FAST_CHARGE_TIME_LEFT,
	SLOW_CHARGE_TIME_LEFT,
	ULT_CHARGE}

# ----------------------------------------------------------------Gamer settings
var low_health := 50 # self-regen stops
var mid_health := 100 # health packs stop
var max_health := 150 # regulators stop

var recharge_rate := 6 # after this many ticks, we decrease the "time" left by 1
var fast_recharge_time := 85
var fast_max_charges := 1
var slow_recharge_time := 255

var ult_charge_max := 250 

# --------------------------------------------------------------------Gamer vars
var state_slice = []

var fast_recharge_ticks_left := 0 # how many phys ticks b4 next time decrease
var slow_recharge_ticks_left := 0 # set=recharge_rate on ability press


# ----------------------------------------------------------------Input settings
var mouse_sensitivity := 0.05
var jump_try_ticks_default := 7

# --------------------------------------------------------------------Input vars
var jump_try_ticks_remaining := 0

# ------------------------------------------------------------------Network vars
var state_buffer:PoolBuffer


# ---------------------------------------------------------------Experiment Vars
var raycast_this_physics_frame = false
var target_start_position = Vector3(10.0, 50.0, 0.0)
var target_position = Vector3(10.0, 5.0, -15.0)
var fire_from_position = Vector3(10.0, 5.0, -25.0)
var test_target:StaticBody
var targets = []

func _ready():
	init_state_recording()
	
	setup_test_targets()
	
	visual_root.set_as_toplevel(true)
	
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

func init_state_recording():
	state_slice = []
	state_slice.resize(STATE.size())
	state_slice[STATE.POSITION] = transform.origin
	state_slice[STATE.VELOCITY] = Vector3()
	state_slice[STATE.HEALTH] = mid_health
	state_slice[STATE.ULT_CHARGE] = 0
	state_slice[STATE.SLOW_CHARGE_TIME_LEFT] = 0
	state_slice[STATE.FAST_CHARGE_TIME_LEFT] = 0

	var state_stubs = []
	state_stubs.resize(STATE.size())
	state_stubs[STATE.POSITION] = PoolVector3Array()
	state_stubs[STATE.VELOCITY] = PoolVector3Array()
	state_stubs[STATE.HEALTH] = PoolByteArray()
	state_stubs[STATE.FAST_CHARGE_TIME_LEFT] = PoolByteArray()
	state_stubs[STATE.SLOW_CHARGE_TIME_LEFT] = PoolByteArray()
	state_stubs[STATE.ULT_CHARGE] = PoolByteArray()
	state_buffer = PoolBuffer.new(state_stubs)

func setup_test_targets():
	"""
	for i in range(10):
			var target_to_shoot = target.instance()
			target_to_shoot.transform.origin = target_position
			targets.push_back(target_to_shoot)
			get_tree().get_root().call_deferred("add_child", target_to_shoot)
	"""
	test_target = target.instance()
	test_target.transform.origin = target_start_position
	get_tree().get_root().call_deferred("add_child", test_target)

func _unhandled_input(event):
	if event.is_action_pressed("click"):
#		test_target.transform.origin = target_position
#		test_target.force_update_transform()
		raycast_this_physics_frame = true
	
	if event.is_action_pressed("signature_ability"):
		pass
		
	if (event is InputEventMouseMotion 
		&& Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED):
		var yaw_delta = -event.relative.x * mouse_sensitivity
		var pitch_delta = event.relative.y * mouse_sensitivity
		yaw += yaw_delta
		yaw = Math.deg_to_deg360(yaw)
		
		rotation_degrees.y = yaw
		orthonormalize()
		visual_root.rotation_degrees.y = yaw
		visual_root.orthonormalize()
		pitch = clamp(
			pitch - pitch_delta, 
			-90.0, 
			90.0
			)
		camera.rotation_degrees.x = pitch
		camera.orthonormalize()
		
	elif event.is_action_pressed("toggle_mouse_mode"):
		var new_mouse_mode
		if Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED:
			new_mouse_mode = Input.MOUSE_MODE_VISIBLE
		elif Input.get_mouse_mode() == Input.MOUSE_MODE_VISIBLE:
			new_mouse_mode = Input.MOUSE_MODE_CAPTURED
		Input.set_mouse_mode(new_mouse_mode)
		
	elif event.is_action_pressed("jump"):
		# try a jump for next jump_try_ticks_default ticks
		jump_try_ticks_remaining = jump_try_ticks_default

func _process(delta):
	var last_physics_tick_origin = state_buffer.read_var(
		STATE.POSITION, Network.physics_tick_id - 1)
	var this_physics_tick_origin = state_buffer.read_var(
		STATE.POSITION, Network.physics_tick_id)
	visual_root.transform.origin = last_physics_tick_origin.linear_interpolate(
		this_physics_tick_origin, Engine.get_physics_interpolation_fraction())

func _physics_process(delta):
	handle_raycast()
	
	record_move()

	record_state()

	handle_networking()

	calculate_movement_4(delta)

	apply_movement()
	
#	test_slide()

	calculate_next_move()

	calculate_next_state()
	
#	print("At %s during frame %d" % [transform.origin, Network.physics_tick_id])

var frames_since_cast = 0

func handle_raycast():
	if raycast_this_physics_frame:
		velocity += 30.0 * -camera.get_global_transform().basis.z
#		velocity.y = 0
#		for i in range(20):
#			move_and_slide(20.0 * -camera.get_global_transform().basis.z, Vector3.UP, false, 2)
#		var space_state = get_world().direct_space_state
#		var target_transform = Transform(
#			test_target.transform.basis, target_position)
#		var tts = PhysicsServer.body_get_direct_state(test_target.get_rid())
#		tts.set_transform(target_transform)
#		var result = space_state.intersect_ray(
#			fire_from_position, 
#			target_position)
#		print(frames_since_cast)
#		frames_since_cast += 1
#		if result and is_instance_valid(result.collider):
#			print(result.collider)
#		var hit_dist
#		for i in range(1):
#			hit_dist = Intersector.intersect_ray_sphere(
#				camera.get_global_transform().origin, 
#				camera.get_global_transform().origin + (40.0 * -camera.get_global_transform().basis.z),
#				Network.sigma_position,
#				1)
#		if hit_dist == -1.0:
#			print("missed aha")
#		else:
#			print("sigma destroyed")
		raycast_this_physics_frame = false
		# for i in range(targets.size()):
		# 	var target_to_shoot = targets[i]
		# 	target_to_shoot.transform.origin = Vector3(-10.0, 2.0, i * -4.0)
		# 	target_to_shoot.force_update_transform()
		# 	var result = space_state.intersect_ray(
		# 		Vector3(10.0, 2.0, 3.0), 
		# 		target_to_shoot.transform.origin)
		# 	if result and is_instance_valid(result.collider):
		# 		print(result.collider)
		# 		#result.collider.queue_free()
		# 	target_to_shoot.transform.origin = target_position

func test_slide():
#	print("Tested slide at frame %d" % Network.physics_tick_id)
	
#	transform.origin = Vector3(0.0, 5.0, 0.0)
#	test_target.transform.origin = Vector3(10.0, 50.5, 0.0)
#	print(is_on_floor())
#	move_and_slide(Vector3(0.0, -100.0, 0.0), Vector3.UP)
#	print(is_on_floor())
#	move_and_slide(Vector3(0.0, -100.0, 0.0), Vector3.UP)
#	print(is_on_floor())
#	move_and_slide(Vector3(0.0, -100.0, 0.0), Vector3.UP)
#	print(is_on_floor())
	for i in range(100):
		move_and_slide(pow(-1, i) * Vector3(20.0, -20.0, 0.0), Vector3.UP, false, 2)

func record_move():
	move_buffer.write(
		move_slice,
		Network.physics_tick_id)

func calculate_next_move(): # get move to process this phys frame
	# --------for calc look_delta
	var last_look = move_slice[MOVE.LOOK]
	
	move_slice[MOVE.PROCESSED] = 1
	
	if jump_try_ticks_remaining > 0:
		move_slice[MOVE.JUMP] = 1
		jump_try_ticks_remaining -= 1
	else:
		move_slice[MOVE.JUMP] = 0

	# --------cardinal direction
	move_slice[MOVE.X_DIR] = 0
	move_slice[MOVE.Z_DIR] = 0
	
	if Input.is_action_pressed("move_left"):
		move_slice[MOVE.X_DIR] -= 1
	if Input.is_action_pressed("move_right"):
		move_slice[MOVE.X_DIR] += 1
	if Input.is_action_pressed("move_forward"):
		move_slice[MOVE.Z_DIR] += 1
	if Input.is_action_pressed("move_backward"):
		move_slice[MOVE.Z_DIR] -= 1
	
	move_slice[MOVE.LOOK] = Vector2(yaw, pitch)
	
	move_slice[MOVE.LOOK_DELTA] = int(
		last_look.is_equal_approx(
			move_slice[MOVE.LOOK]))

func record_state(): # state @start of phys frame, before move/other changes
	state_buffer.write(
		state_slice,
		Network.physics_tick_id)

func calculate_next_state():
	var fast_time_left = state_slice[STATE.FAST_CHARGE_TIME_LEFT] 
	if fast_time_left > 0:
		if fast_recharge_ticks_left == 0:
			state_slice[STATE.FAST_CHARGE_TIME_LEFT] -= 1
			fast_recharge_ticks_left = recharge_rate
		else:
			fast_recharge_ticks_left -= 1

	var slow_time_left = state_slice[STATE.SLOW_CHARGE_TIME_LEFT]
	if slow_time_left > 0:
		if slow_recharge_ticks_left == 0:
			state_slice[STATE.SLOW_CHARGE_TIME_LEFT] -= 1
			slow_recharge_ticks_left = recharge_rate
		else:
			slow_recharge_ticks_left -= 1

	state_slice[STATE.POSITION] = transform.origin

	state_slice[STATE.VELOCITY] = velocity

func get_fast_charges_stored() -> int:
	var fast_charge_time_stored:int = (
		(fast_recharge_time * fast_max_charges) - 
		state_slice[STATE.FAST_CHARGE_TIME_LEFT])
	
	return fast_charge_time_stored / fast_recharge_time

func handle_networking():
	pass
