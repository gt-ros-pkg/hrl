Header header
# seq is [0, 1, ...]
# frame_id in header is base_frame

sensor_msgs/PointCloud2 pc_capture
# in PC's native frame

string[] frame_names
# names matching respective saved_frames

geometry_msgs/PoseStamped[] saved_frames
# header is same as total outer header

geometry_msgs/PoseStamped pc_frame
