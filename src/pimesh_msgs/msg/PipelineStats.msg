# What one stage of the pipeline measured about itself over the last window.
#
# Every node publishes one of these on /pipeline/stats; the dashboard subscribes
# and draws them and computes nothing of its own. That rule is what makes this
# message the contract rather than a debug aid: a number on screen is a number
# some node measured, so there is exactly one place to go when it looks wrong.

# `stamp` is when the window closed, and it is the *only* time the dashboard is
# allowed to reason about — staleness is measured on receipt, never on a stamp.
std_msgs/Header header

# The stage's name, matching the node: capture, decode, keypoints, depth,
# fusion, mesh. A string rather than an enum because the set grows and a
# dashboard that shows an unknown stage as its own row is better than one that
# shows it as UNKNOWN=7.
string stage

# --- Throughput -------------------------------------------------------------

float32 rate_hz                 # completions per second over the window
float32 latency_ms              # mean cost of one item, on the node's own clock
float32 latency_p95_ms          # the tail, which is what stalls a pipeline

# --- Drops, and there are two kinds ------------------------------------------
#
# Keeping these apart is not bookkeeping. depth_node drops ~72% of the frames
# offered to it because it runs at 13 Hz against a 47 Hz camera, and that is the
# single-slot mailbox working exactly as designed. A frame lost to QoS on the
# wire is a fault. One counter for both would make the healthy pipeline and the
# broken one look identical.

uint64 frames_in                # offered to this stage since it started
uint64 frames_out               # published by it
uint64 dropped_by_design        # mailbox overwrites: the newest frame won
uint64 dropped_in_transport     # QoS loss, deserialisation failure, malformed

# Free-form, for the one fact a stage has that no other stage does — the
# execution provider depth_node got, the odometry regime keypoint_node is in.
# Displayed verbatim; never parsed.
string detail
