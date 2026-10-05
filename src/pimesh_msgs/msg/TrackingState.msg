# Whether the tracker trusts the pose it published at this stamp (#13's P19).
#
# One of these is published by odometry_node for every depth frame it handles,
# **before** the `odom -> base_link` transform at the same stamp, and fusion_node
# looks it up by that exact stamp before integrating the frame.
#
# **Not carried by withholding the transform.** The obvious way to say "do not
# fuse this frame" is to publish no pose for it — and tf2 then *interpolates*
# across the missing stamp from the good poses either side, so fusion_node would
# find a pose and integrate the frame anyway. A refusal has to be a value.

std_msgs/Header header

# **Zero is UNKNOWN, not OK.** A default-constructed message, a field nobody set
# and a state that was never received must not have the same spelling as a pose
# that is fine — fusion_node refuses UNKNOWN exactly as it refuses LOST.
uint8 UNKNOWN = 0
uint8 OK = 1
uint8 LOST = 2
uint8 state

# Whether *this* frame's pose came from a fit (true) or was held from the last
# one (false). OK with posed=false is a hold the tracker is still tolerating; it
# becomes LOST after `lost_after_holds` of them in a row.
bool posed

# Consecutive depth frames without a fit, ending at this one. 0 when posed.
uint32 holds
