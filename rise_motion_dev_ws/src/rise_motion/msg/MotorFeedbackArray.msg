# Array of actual motor states
# Each element corresponds to one motor's feedback

int32[] actual_positions      # Actual position in encoder steps
uint16[] status_words         # CiA 402 status word for each motor
int32[] velocity              # Actual velocity (optional, if available)
