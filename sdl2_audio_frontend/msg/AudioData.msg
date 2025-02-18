std_msgs/Header header
uint32 sample_rate    # Hz
uint8 format         # Format identifier (e.g., 0 for F32, 1 for S16)
uint8 channels      # Number of channels
uint32 samples      # Number of samples in the data array
uint8[] data        # Raw audio data