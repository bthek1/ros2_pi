# ORB features found in one frame, with enough in them for a consumer to draw
# tracks without re-running the detector.
#
# Struct-of-arrays, not an array of a Keypoint message. Two reasons, and the
# second is the one that matters here: a `Keypoint[]` of 500 entries is 500
# separate CDR-encoded submessages, where these parallel arrays are ten
# contiguous blocks that memcpy; and the descriptors are a flat byte blob either
# way, so splitting them per-feature would have bought nothing but a length
# prefix each. Every array below has the same length except `descriptors`.
#
# `prev_x`/`prev_y` were added 2026-09-23, when the pose estimator moved out of
# keypoint_node into odometry_node: the geometry's pairing is the detector's to
# report, and a consumer that reconstructed it from `track_id` would silently
# get a looser one. See the field comment at the foot of this file.

# The frame these were found in: `stamp` is the capture time of the source
# image, never the moment the detector finished. Derived data keeps the header
# of what it describes.
std_msgs/Header header

# The image the pixel coordinates below are in. A consumer that draws on a
# downscaled preview needs this to scale; one that unprojects needs it to check
# the CameraInfo it holds is for the same resolution.
uint32 image_width
uint32 image_height

# --- Per-feature, all of these the same length -----------------------------

float32[] x                     # pixels, origin top-left
float32[] y
float32[] size                  # diameter of the meaningful neighbourhood, px
float32[] angle                 # dominant orientation, degrees, -1 if undefined
float32[] response              # detector score; higher is a stronger corner

# Track identity, and the reason this message exists rather than a bare point
# cloud. Every feature carries the id of the track it belongs to, **including on
# the frame where that track is first seen**. Ids are unique within a session,
# are never reused, and are always >= 0.
#
# **It used to be -1 on a first sighting, and that was one field saying two
# things.** `odometry_node` intersects two frames' landmarks by id, so a feature
# whose id was hidden on the frame a keyframe was taken from could never be
# matched against the same feature later — it would silently contribute no
# landmark, on the ~10% of features that are new in any given frame. Splitting
# the two facts apart is also what this project's own record keeps asking for:
# see the `-1` covariance sentinel and the `cost_mean=0.00` gate, both of which
# were one value spelled the same as a different value.
int32[] track_id

# True where this frame is the first sighting of that track. What a consumer
# drawing tracks wants, and what `track_id == -1` used to encode.
bool[] is_new

# --- Descriptors ------------------------------------------------------------

# ORB is a 256-bit binary descriptor, so 32 bytes per feature. The field is here
# rather than hard-coded because the matcher's Hamming threshold is quoted in
# bits (64 of 256) and a consumer has to know the denominator.
uint32 descriptor_bytes

# Row-major, `descriptor_bytes` per feature, `descriptor_bytes * len(x)` total.
uint8[] descriptors

# --- Where each feature was one frame ago -----------------------------------

# The pixel this feature was mutually best-matched to in the *immediately
# previous* frame, or NaN where it was not matched to one. Parallel to `x`/`y`,
# so it is the same length as every other per-feature array above.
#
# **This is a different matching from `track_id` and the difference is the
# point.** A track id comes from a pooled pass over a window of the last ten
# frames, which forgives detection churn; these come from a mutual-best pass
# against the previous frame alone, which is stricter. The geometry wants pairs
# exactly one frame apart — a match five frames back spans five times the motion
# and would be weighted as though it spanned one — so a consumer must not
# reconstruct these by intersecting two frames' `track_id` arrays. It would get
# a different, looser pairing, and a rotation fit on looser pairs does not fail:
# it returns a wrong answer with a plausible residual.
#
# Pixels rather than indices into the previous message, so a consumer never has
# to hold the previous frame or keep its own copy of the tracker's window.
float32[] prev_x
float32[] prev_y
