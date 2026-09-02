# One frame's ORB features and their matches into recent frames.
#
# The header is the *input frame's* — stamp and camera_optical_frame — because
# these keypoints describe that frame, not the moment detection finished.
#
# Parallel arrays, all of length `count`. Flat arrays rather than an array of
# a Keypoint message: 500 features per frame at camera rate, and a nested type
# costs an allocation each. `descriptors` is count*descriptor_size bytes,
# row-major, one row per feature.

std_msgs/Header header

# Size of the image the features were found in, so a consumer can normalise
# without needing /camera_info.
uint32 image_width
uint32 image_height

uint32 count

float32[] x            # pixel coordinates, top-left origin
float32[] y
float32[] size         # feature diameter in pixels
float32[] angle        # dominant orientation, degrees, -1 if not computed
float32[] response     # detector strength, for ranking at the feature cap

# Match into the previous frame: index into that frame's arrays, or -1 when
# this feature went unmatched. Matching is pooled over a window of recent
# frames (see keypoint_node), so an unmatched feature is not necessarily new.
int32[] match_index

# Persistent id for a feature followed across frames, -1 when untracked.
# Stable ids are what let the dashboard draw tracks without rematching.
int32[] track_id

# Descriptor rows. ORB is 32 bytes of binary descriptor per feature; matching
# is Hamming distance over 256 bits.
uint16 descriptor_size
uint8[] descriptors
