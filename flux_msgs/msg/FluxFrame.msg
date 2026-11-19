# One flux frame as a bag records it. `data` holds the frame bytes in the codec named by
# `codec`; for raw it is the flux wire bytes, laid out by `fingerprint`.
std_msgs/Header header
string flux_topic
uint64 fingerprint
string codec
uint8[] meta
uint8[] data
# Fields from here on are new in format 1 and absent before it: a reader of an older bag reads
# version 0, and a reader older than these fields ignores them. A reader refuses a later version.
uint32 format_version
# The ROS type `fingerprint` names (`pkg/msg/Name`), "" when the recorder had none installed.
string ros_type
