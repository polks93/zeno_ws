Header header

uint8 sequence

sensor_msgs/Image current_image #the current image

geometry_msgs/Vector3 current_image_nav_status #the latitude, longitude position of the first image (provided by the navigation filter)

geometry_msgs/Vector3 current_image_orientation #the orientation of the current image (roll, pitch, yaw)

geometry_msgs/Vector3 distance_previous_image #the distance computed from the previous image

std_msgs/Bool cv_algorithm_status #the status of Computer Vision algorithm. If true has been computed and its results are deemed as good.

