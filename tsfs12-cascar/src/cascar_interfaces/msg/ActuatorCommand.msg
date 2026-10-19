std_msgs/Header header

# Dimensionless actuator-effort requests. These are not velocity targets.
# Consumers clamp values to [-1, 1]. Positive steering turns left (CCW).
float64 throttle
float64 steering