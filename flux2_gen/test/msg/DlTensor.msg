# DLPack's DLTensor as tensor_msgs/ExperimentalTensor lays it out, under another name.
uint8  dtype_code
uint8  dtype_bits
uint16 dtype_lanes
int64[]  shape
int64[]  strides
uint64   byte_offset
uint8[] data
