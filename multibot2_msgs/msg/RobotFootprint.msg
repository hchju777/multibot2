# Footprint Type
uint8 POINTROBOT=1 
uint8 CIRCULARROBOT=2 
uint8 LINEROBOT=3 
uint8 PILLROBOT=4 
uint8 POLYGONROBOT=5 

int64 type

bool holonomic

# Parameter for LineRobot, PillRobot
geometry_msgs/Point point1

# Parameter for LineRobot, PillRobot
geometry_msgs/Point point2

# Radius of point1
# Parameter for CircularRobot, PillRobot
float64 radius

# Robot footprint (polygon descriptions)
geometry_msgs/Polygon polygon