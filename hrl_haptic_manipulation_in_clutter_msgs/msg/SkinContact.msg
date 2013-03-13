Header header
geometry_msgs/Point[] locations
geometry_msgs/Vector3[] forces

# normals to the manipulator surface at the points of contact.
geometry_msgs/Vector3[] normals

string[] link_names

#all the points (from collision map) that make up one contact location.
hrl_msgs/FloatArrayBare[] pts_x
hrl_msgs/FloatArrayBare[] pts_y
hrl_msgs/FloatArrayBare[] pts_z




