Header header

uint8 sequence

marta_msgs/Position position

marta_msgs/Position pre_position	# Previous waypoint

marta_msgs/Euler orientation

marta_msgs/Position position_tolerance

marta_msgs/Euler orientation_tolerance

# reference is to be considered as thrust if thrust == 1
# reference is to be considered as speed if thrust == 0

geometry_msgs/Vector3 reference

geometry_msgs/Vector3 reference_end

string mission

uint8 thrust

uint8 mode

uint8 MODE_ALTITUDE = 1
uint8 MODE_DEPTH = 0

