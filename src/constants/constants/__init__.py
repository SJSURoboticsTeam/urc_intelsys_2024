"""
This file contains miscellaneous constants relevant throughout the project.
"""

# Sensor topics
COMPASS_TOPIC = "/sensors/compass"
GPS_TOPIC = "/sensors/gps"
RGB_CAMERA_TOPIC = "/camera/image_raw"
DEPTH_CAMERA_TOPIC = "/camera/depth/image_raw"

# In-between topics
GOAL_TOPIC = "/goal"
MAP_TOPIC = "/map"

# misc constants
QOS = 10  # quality of service, used when creating publishers/subscribers

UPDATE_FREQ = 0.1
