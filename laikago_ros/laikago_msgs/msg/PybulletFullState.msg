float32[3] position	# body pos
float32[4] orientation	# body orientation (quaternion)
float32[3] rpy  	# roll pitch yaw
float32[9] rBody	# flattened orientation matrix (column rank)
float32[3] vBody 	# body frame lin vel
float32[3] omegaBody	# body frame ang vel
float32[3] vWorld	# world frame lin vel
float32[3] omegaWorld	# world frame ang vel
LegCmdArray legCmdArray	# impedance control leg commands
float32[3] v_des	# desired body frame velocity in xyz
float32 yaw_rate_des	# desired yaw rate
MotorState[12] motorState	# full motor states
float32[8] rlFootPos	# desired foot positions for gait, from RL
uint8 rlFlag		# whether rlFootPos has any data
