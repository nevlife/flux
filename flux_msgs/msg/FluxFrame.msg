# One flux frame as a bag records it. `data` holds the frame bytes in the codec named by
# `codec`; for raw it is the flux wire bytes, laid out by `fingerprint`.
std_msgs/Header header
string flux_topic
uint64 fingerprint
string codec
uint8[] meta
uint8[] data
