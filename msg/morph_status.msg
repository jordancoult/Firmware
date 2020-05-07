uint64 timestamp  # for UORB - time since system start (microseconds)
uint8 mode
float32[4] angles
float32[2] raw_cg
float32[2] filt_cg
float32[2] ct
bool shutter_open