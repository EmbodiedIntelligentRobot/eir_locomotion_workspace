# Policy-ready normalized depth, ordered oldest to newest.
# header.stamp is the acquisition time of the newest source frame.
std_msgs/Header header

uint32 frame_count
uint32 height
uint32 width
float32[] data

# Monotonic sequence assigned by the depth pipeline.
uint64 source_sequence

# Versioned preprocessing contract, for example "depth_v1_masked".
string preprocessing_version
