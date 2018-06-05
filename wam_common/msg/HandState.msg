# This is a modification of sensor_msgs::JointState to include tactile data.

Header header

string[] name
float64[] position
# float64[] velocity
float64[] effort
TactileArray tactile_array
ForceTorqueSensor force_torque_sensor