Header header
int32 seq
uint32 identifier
uint32 purpose

#Purpose == 0 request info
uint32 reqIdentifier
uint32[] reqBow
uint32[] reqKey

#Purpose == 1 BoW descriptor 
uint32[] matrix_dims
float64[] matrices_flat
float32[] key_vec

#Purpose == 2 Keyframe
sensor_msgs/PointCloud2 cloud
geometry_msgs/Pose pose

#Purpose == 3 Trajectory update
uint32 trajectoryIter
geometry_msgs/Pose[] poses


