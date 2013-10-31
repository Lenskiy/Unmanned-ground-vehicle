# Possible object types
uint16 OBSTACLE_UNKNOWN = 0
uint16 OBSTACLE_BARRICADE = 1
uint16 OBSTACLE_CONE = 2
uint16 OBSTACLE_PEDESTRIAN = 3
uint16 OBSTACLE_SPEEDBUMP = 4
uint16 OBSTACLE_VEHICLE = 5

uint16 LIGHT_ARROW_LEFT = 10
uint16 LIGHT_ARROW_RIGHT = 11
uint16 LIGHT_GREEN = 12
uint16 LIGHT_RED = 13

uint16 MARK_STOPLINE = 20

uint16 SIGN_CROSSROAD = 30
uint16 SIGN_PEDESTRIAN_ZONE = 31
uint16 SIGN_ROAD_WORKS = 32
uint16 SIGN_SPEED_MAXIMUM = 33
uint16 SIGN_SPEED_MINIMUM = 34

std_msgs/Header header
# Unique ID
uint32 id
# One of the types specified above
uint16 type
# Additional data, e.g. the actual speed limit
uint32 data
# 6D pose of the object
geometry_msgs/Pose pose
