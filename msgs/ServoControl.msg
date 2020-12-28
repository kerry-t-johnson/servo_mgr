# The ServoControl message is commonly used message in this package to
# assign a value to a specific servo.
#
# The purpose of the value is dependent on the topic or service being
# called (e.g. may be interpreted as absolute [0, 4096] or
# proportional [-1.0, 1.0])

int16 servo_id
float32 value
