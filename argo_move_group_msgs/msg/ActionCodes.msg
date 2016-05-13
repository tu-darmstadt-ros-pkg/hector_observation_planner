uint8 val

uint8 SAMPLE = 0    # sample camera poses for target
uint8 PLAN_BASE = 1 # sample camera poses for target and plan a promising base position
uint8 SAMPLE_MOVE_ARM = 2 # sample camera poses for target and try to move arm to the best sample 
uint8 MOVE_ARM = 3  # pop next best sample from list and try to move arm to this pose