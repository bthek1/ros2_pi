# ORB features found in one frame, with enough in them for a consumer to draw
# tracks without re-running the detector.
#
# Struct-of-arrays, not an array of a Keypoint message. Two reasons, and the
# second is the one that matters here: a `Keypoint[]` of 500 entries is 500
# separate CDR-encoded submessages, where these parallel arrays are ten
# contiguous blocks that memcpy; and the descriptors are a flat byte blob either
# way, so splitting them per-feature would have bought nothing but a length
# prefix each. Every array below has the same length except `descriptors`.

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
# cloud. A feature matched to a track the pipeline is already following carries
# that track's id; -1 means this frame is the first time it has been seen. Ids
# are unique within a session and are never reused.
int32[] track_id

# --- Descriptors ------------------------------------------------------------

# ORB is a 256-bit binary descriptor, so 32 bytes per feature. The field is here
# rather than hard-coded because the matcher's Hamming threshold is quoted in
# bits (64 of 256) and a consumer has to know the denominator.
uint32 descriptor_bytes

# Row-major, `descriptor_bytes` per feature, `descriptor_bytes * len(x)` total.
uint8[] descriptors
