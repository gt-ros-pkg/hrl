Header header
float32 time_offset
geometry_msgs/TransformStamped tool_frame

# pointcloud data
int32 pc_ind
geometry_msgs/Vector3 pc_normal

# wrench data
geometry_msgs/Wrench wrench
float32 force_magnitude
float32 force_normal
float32 force_tangental

# semantic data
string region
string user
int32 contact_period
float32 time_from_contact_start
