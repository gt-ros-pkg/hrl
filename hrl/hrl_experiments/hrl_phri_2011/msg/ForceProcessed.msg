Header header
float32 time_offset
geometry_msgs/TransformStamped tool_frame

# pointcloud data
int32 pc_ind
geometry_msgs/Point pc_pt
float32 pc_dist
geometry_msgs/Vector3 pc_normal

# wrench data
geometry_msgs/Wrench wrench
geometry_msgs/Wrench raw_wrench
float32 force_magnitude
float32 force_normal
float32 force_tangental

# semantic data
geometry_msgs/Point ell_coords
geometry_msgs/Transform tool_ell_frame
string region
string user
int32 contact_period
float32 time_from_contact_start
