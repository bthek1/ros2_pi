"""How many depth maps arrived with their RGB twin — the pure half of the probe.

Its own module, with no ROS import in it, so it can be tested without a running
graph. That matters because **this is the part that was wrong twice**, both
times in the same shape: the number being computed described the PROBE rather
than the pipeline.

* The first version compared the whole stamp sets. The probe subscribes from
  outside the container with a bounded queue and services one callback per
  spin, so it drops independently on each topic and routinely holds a stamp
  from one and not the other. It saw 9 depth against 11 rgb messages, from
  different subsets, and reported the gap as a fault of the node.
* The second version stopped collecting the moment its byte comparisons were
  done, truncating both sets mid-stream and leaving the frame at the boundary
  looking orphaned.

The fix for both is the same: judge only the INTERIOR of the observed window. A
depth stamp outside the range of rgb stamps actually seen was published before
this probe was listening or after it stopped, and its twin was never in scope.
Judging those is judging the probe's own start and stop times.
"""

# How many of the depth maps seen must also arrive as /depth/rgb. Not 100%,
# because even over the interior the two subscriptions drop independently: a
# stamp missing from one is at least as likely to be the probe's loss as the
# node's. A node publishing depth with no twin at all scores 0% and is what
# this catches. The EXACTNESS claim rides on the byte comparison instead,
# which no amount of dropping can fake.
MIN_PAIRED = 0.9


def pairing(depth_stamps, rgb_stamps):
    """Return `(seen, paired, ratio)` over the interior of the observed window."""
    if not rgb_stamps:
        return 0, 0, 0.0
    lo, hi = min(rgb_stamps), max(rgb_stamps)
    interior = {k for k in depth_stamps if lo <= k <= hi}
    seen = len(interior)
    paired = seen - len(interior - rgb_stamps)
    return seen, paired, (paired / seen if seen else 0.0)
