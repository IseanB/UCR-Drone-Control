# State: GROUND_IDLE = 0, LIFTING_OFF = 1, HOVERING = 2, IN_TRANSIT = 4, LANDING = 8, SHUTTING_DOWN = 16
uint16 droneid
uint8 state
geometry_msgs/Pose pose
geometry_msgs/Pose target