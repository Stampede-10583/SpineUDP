# Optional: published by the upstream adapter node on spine/adapter_stats, if it chooses to.
# spine_stats forwards frames_unpaired to NetworkTables as Spine/Orin/framesUnpaired.

builtin_interfaces/Time stamp
uint64 frames_paired    # frames given a controller_timestamp
uint64 frames_unpaired  # frames dropped for having no matching trigger
