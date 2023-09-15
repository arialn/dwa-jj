define hokuyolaser ranger
(
	sensor
	( 			
		# laser-specific properties
		# factory settings for LMS200	
		range [ 0.0 5.0 ]
		fov 360.0
		samples 360 #360 1080 1800 2520 3240
	)

	# generic model properties
	color "blue"
	size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet	
)



