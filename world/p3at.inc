include "hokuyo.inc"

define pioneer3at position
(
	color "red"          # Default color.
	drive "diff"         # Differential steering model.diff or omni
	gui_nose 1           # Draw a nose on the robot so we can see which way it points
	obstacle_return 1    # Can hit things.
	laser_return 1	            	# reflects sonar beams


	# alternative odometric localization with simple error model
	localization "gps"                 # Change to "gps" to have impossibly perfect, global odometry
#	odom_error [ 0.05 0.05 0.1 0 ]      # Odometry error or slip in X, Y and Theta (Uniform random distribution)   
	odom_error [ 0 0 0 0 ]

	# four DOF kinematics limits
	# [ xmin xmax ymin ymax zmin zmax amin amax ]				
	#velocity_bounds     [-0.75 10 0 0 0 0 -140.0 140.0 ]					
	#acceleration_bounds [-1 1 0 0 0 0 -183.0 183.0 ]
	velocity_bounds     [-10 10 -10 10 0 0 -140.0 140.0 ]					
	acceleration_bounds [-10 10 -10 10 0 0 -183.0 183.0 ]

	# Actual size
	size [0.45 0.4 0.25]

	# The pioneer's center of rotation is offset from its center of area
	origin [0 0 0 0]

	# Estimated mass in KG
	mass 40.0 

	# Body shape:
#	block
#	( 
#		points 8
#		point[0] [-0.18 0.313]
#		point[1] [0.18 0.313]
#		point[2] [0.2525 0.18]
#		point[3] [0.2525 -0.18]
#		point[4] [0.18 -0.313]
#		point[5] [-0.18 -0.313]
#		point[6] [-0.2525 -0.18]
#		point[7] [-0.2525 0.18]
#		z [0 1]
#	)
	hokuyolaser( pose [0.0 0 -0.0 0 ])
)

define ob position
(
	color "blue"          # Default color.
	drive "diff"         # Differential steering model.diff or omni
	gui_nose 1           # Draw a nose on the robot so we can see which way it points
	obstacle_return 1    # Can hit things.
	laser_return 1	            	# reflects sonar beams


	# alternative odometric localization with simple error model
	localization "gps"                 # Change to "gps" to have impossibly perfect, global odometry
#	odom_error [ 0.05 0.05 0.1 0 ]      # Odometry error or slip in X, Y and Theta (Uniform random distribution)   
	odom_error [ 0 0 0 0 ]

	# four DOF kinematics limits
	# [ xmin xmax ymin ymax zmin zmax amin amax ]				
	#velocity_bounds     [-0.75 10 0 0 0 0 -140.0 140.0 ]					
	#acceleration_bounds [-1 1 0 0 0 0 -183.0 183.0 ]
	velocity_bounds     [-10 10 -10 10 0 0 -140.0 140.0 ]					
	acceleration_bounds [-10 10 -10 10 0 0 -183.0 183.0 ]

	# Actual size
	size [1.0 1.0 1.0]

	# The pioneer's center of rotation is offset from its center of area
	origin [0 0 0 0]

	# Estimated mass in KG
	mass 40.0 
)

