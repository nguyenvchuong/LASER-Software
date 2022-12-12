float32[3] v_des	# desired body frame velocity in xyz
float32 yaw_rate_des	# desired yaw rate
float32[8] rlFootPos	# desired foot positions for gait, from RL
uint8 yawRateFlag	# whether RL is choosing yaw rate
uint8 velFlag		# whether RL is choosing v_des
