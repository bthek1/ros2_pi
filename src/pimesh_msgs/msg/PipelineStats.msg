# What one pipeline stage measured about itself over the last window.
#
# Every node publishes this for its own stage on a shared topic, so the
# dashboard and `ros2 topic echo` see the same numbers the node saw. The
# dashboard never computes: if a figure is on screen, a node measured it.

std_msgs/Header header

string stage           # "capture", "decode", "keypoints", "depth", "fusion", "mesh"

float32 rate_hz        # averaged over the node's stats window
float32 latency_ms     # mean per-frame cost, measured on this node's own clock
float32 latency_p95_ms

uint64 processed

# Two different kinds of loss, and only the second one is a fault.
#   dropped_mailbox   — dropped by design: a newer frame overwrote an
#                       unprocessed one. depth_node dropping ~70% of frames at
#                       13 Hz against a 47 fps camera is the design working.
#   dropped_transport — QoS loss, decode failures, malformed frames.
uint64 dropped_mailbox
uint64 dropped_transport

# True when nothing has arrived for the stage's stale_after window. Measured
# on RECEIPT time, never on header.stamp — see CLAUDE.md on stamp lag.
bool stale

string detail          # free-form: inference provider, voxel count, regime
