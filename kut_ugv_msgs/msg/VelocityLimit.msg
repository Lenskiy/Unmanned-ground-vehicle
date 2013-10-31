# Limit maximum velocity at given point

# Possible action types
uint16 NONE = 0
uint16 STEER_LEFT = 1
uint16 STEER_RIGHT = 2

# if (max_velocity <= DBL_EPSILON), i.e. equal to 0.0, then vehicle has to stop at x,y
# if (is_nan(x) || is_nan(y)) == true, then velocity limit has imediate effect
float64 x
float64 y
float64 max_velocity
# Action type
uint16 type
