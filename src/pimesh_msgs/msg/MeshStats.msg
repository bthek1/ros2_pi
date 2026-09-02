# What the last surface extraction produced.

std_msgs/Header header

uint32 triangles       # as published on /world/mesh, after decimation
uint32 vertices
uint32 triangles_full  # before decimation; the saved PLY keeps this detail

uint32 components      # connected components surviving the noise prune
uint32 open_loops      # boundary loops left open — frontier, not holes
uint32 filled_loops    # interior loops closed by fan fill

float32 mesh_ms        # wall time of the extraction, snapshot to publish
uint64 voxel_blocks    # allocated blocks in the TSDF at extraction time

geometry_msgs/Point bounds_min
geometry_msgs/Point bounds_max
