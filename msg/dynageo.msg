# Robodub 6/10/20

uint64 timestamp
# Mixer values
float32[4] roll # dynamic geometry condensed mixer vals -- roll scaling
float32[4] pitch # dynamic geometry condensed mixer vals -- pitch scaling
uint8 failed_arm

# Enums
uint8 FAILED_ARM_NONE = 4
# The following are used for multiple variables
uint8 FRONT_RIGHT = 0
uint8 REAR_LEFT = 1
uint8 FRONT_LEFT = 2
uint8 REAR_RIGHT = 3
