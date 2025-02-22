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

# Camera topics
DETECTION_TOPIC = "/detections"
IMAGE_TOPIC = "/camera_image"

# misc constants
QOS = 10  # quality of service, used when creating publishers/subscribers

UPDATE_FREQ = 1
