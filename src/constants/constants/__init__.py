"""
This file contains miscellaneous constants relevant throughout the project.
"""

# Sensor topics
COMPASS_TOPIC = "/sensors/compass"
GPS_TOPIC = "/sensors/gps"

# In-between topics
MAP_TOPIC = "/map"
CARTESIAN_TOPIC = "/cartesian"

# goal topics
GOAL_TOPIC = "/goal"

# PATH TOPIC
PATH_TOPIC = "/path"

DEFAULT_FRAME = "world"
# misc constants
QOS = 10  # quality of service, used when creating publishers/subscribers

UPDATE_FREQ = 1
