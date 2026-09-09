# The state of the reconstruction, published by mesh_node whenever it re-meshes.
#
# It is deliberately separate from PipelineStats: those are rates over a window
# and this is a description of a thing that exists. Merging them would make the
# dashboard poll a 0.1 Hz stage at 10 Hz to read a triangle count.

std_msgs/Header header          # frame_id is the map frame the extents are in

uint32 vertices
uint32 triangles

# Voxels the volume is actually holding, and how many of those passed the
# weight threshold to be meshed. The gap between the two is the noise floor
# doing its job — a volume where they are equal is one that is meshing junk.
uint64 voxels_allocated
uint64 voxels_meshed

# The axis-aligned bounds of the meshed surface, in the header's frame. Metres.
geometry_msgs/Point volume_min
geometry_msgs/Point volume_max

float32 voxel_size_m            # so a viewer can state the resolution honestly
float32 mesh_duration_ms        # the last extraction, off the integration path
uint64 frames_integrated        # posed depth maps folded in since the last reset
