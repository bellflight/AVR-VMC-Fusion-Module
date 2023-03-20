ORIGIN = (32.808549, -97.156345, 161.5)
"""
Origin used for NED to Lat/Lon/Alt conversion
Bell HQ VIP helipad
https://www.google.com/maps/place/32%C2%B048'30.8%22N+97%C2%B009'22.8%22W
"""

HIL_GPS_CONSTANTS = {
    "fix_type": 3,  # 3D fix
    "eph": 20,  # GPS HDOP horizontal dilution of position
    "epv": 5,  # GPS VDOP vertical dilution of position
    "satellites_visible": 13,  # Arbitrary, but for the last 2 years, we've trained teams to look for this 13.
}
"""
Constants for the HIL GPS Mavlink message
https://mavlink.io/en/messages/common.html#HIL_GPS
"""

COURSE_THRESHOLD = 10
"""
Minimum threshold at which the groundspeed is high enough that we decided to publish
a course.
"""

POS_DETLA_THRESHOLD = 10
"""
Minimum threshold at which the normal vector of tracking camera position difference
from the AprilTag position is high enough that we decide to publish a resync delta.
"""

HEADING_DELTA_THRESHOLD = 5
"""
Minimum threshold at which the heading of tracking camera is different enough
AprilTag that we decide to publish a resync delta.
"""

POS_D_THRESHOLD = 30
"""
Minimum threshold at which the vertical position of tracking camera is different
enough from the AprilTag that we discard the AprilTag data.
"""

AT_DERIV_THRESHOLD = 10
"""
Maximum threshold of the normal vector of the derivative of the AprilTag position
that we will still publish a resync delta.
"""
