import math
import time
from typing import List, Tuple

import config
import numpy as np
import pymap3d
from bell.avr.mqtt.module import MQTTModule
from bell.avr.mqtt.payloads import (
    AVRAprilTagsVehiclePosition,
    AVRFusionAttitudeEulerRadians,
    AVRFusionAttitudeQuaternion,
    AVRFusionClimbRate,
    AVRFusionCourse,
    AVRFusionGroundspeed,
    AVRFusionHeading,
    AVRFusionHILGPSMessage,
    AVRFusionPositionGlobal,
    AVRFusionPositionLocal,
    AVRFusionVelocity,
    AVRVIOAttitudeEulerRadians,
    AVRVIOAttitudeQuaternion,
    AVRVIOHeading,
    AVRVIOPositionLocal,
    AVRVIOResync,
    AVRVIOVelocity,
)
from bell.avr.utils.decorators import run_forever, try_except
from loguru import logger


class FusionModule(MQTTModule):
    def __init__(self):
        super().__init__()

        # on_apriltag storage
        self.last_pos: List[float] = [0, 0, 0]
        self.deriv: List[float] = [0, 0, 0]
        self.last_apriltag = time.time()

        self.topic_callbacks = {
            "avr/vio/position/local": self.fuse_pos,
            "avr/vio/attitude/euler/radians": self.fuse_att_euler,
            "avr/vio/heading": self.fuse_att_heading,
            "avr/vio/velocity": self.fuse_vel,
            "avr/fusion/position/local": self.local_to_geo,
            # uncomment to re-enable position re-syncing
            # currently not well enough tested/reliable to be competition ready
            # "avr/apriltags/selected": self.on_apriltag_message
        }

    @try_except(reraise=True)
    def local_to_geo(self, payload: AVRFusionPositionLocal) -> None:
        """
        Callback for the fusion/pos topic. This method calculates the
        geodetic location from an NED position and origin and publishes it.
        """
        lla: Tuple[float, float, float] = pymap3d.enu.enu2geodetic(
            float(payload.e) / 100,  # East   | Y
            float(payload.n) / 100,  # North  | X
            -1 * float(payload.d) / 100,  # Up     | Z
            *config.ORIGIN,
            deg=True,
        )

        self.send_message(
            "avr/fusion/position/global",
            AVRFusionPositionGlobal(lat=lla[0], lon=lla[1], alt=lla[2]),
        )

    @try_except(reraise=True)
    def fuse_pos(self, payload: AVRVIOPositionLocal) -> None:
        """
        Callback for receiving pos data in NED reference frame from VIO and
        publishes into a fusion/pos topic.

        AVR doesn't have sophisticated fusion yet, so this just re-routes the
        message onto the fusion topic.
        """
        self.send_message(
            "avr/fusion/position/local",
            AVRFusionPositionLocal(n=payload.n, e=payload.e, d=payload.d),
        )

    @try_except(reraise=True)
    def fuse_vel(self, payload: AVRVIOVelocity) -> None:
        """
        Callback for receiving vel data in NED reference frame from VIO and
        publishes into a fusion/vel topic.

        AVR doesn't have sophisticated fusion yet, so this just re-routes the
        message onto the fusion topic.
        """
        # forward ned velocity message
        self.send_message(
            "avr/fusion/velocity",
            AVRFusionVelocity(Vn=payload.Vn, Ve=payload.Ve, Vd=payload.Vd),
        )

        # logger.debug("avr/fusion/velocity/ned message sent")

        # compute groundspeed
        gs = np.linalg.norm([payload.Vn, payload.Ve])
        self.send_message(
            "avr/fusion/groundspeed", AVRFusionGroundspeed(groundspeed=float(gs))
        )

        # arctan gets real noisy when the values get small, so we just lock course
        # to heading when we aren't really moving
        if gs >= config.COURSE_THRESHOLD:
            course = math.atan2(payload.Ve, payload.Vn)
            # wrap [-pi, pi] to [0, 360]
            if course < 0:
                course += 2 * math.pi

            # rad to deg
            self.send_message(
                "avr/fusion/course", AVRFusionCourse(course=math.degrees(course))
            )

        m_per_s_2_ft_per_min = 196.85
        self.send_message(
            "avr/fusion/climb_rate",
            AVRFusionClimbRate(climb_rate=-1 * payload.Vd * m_per_s_2_ft_per_min),
        )

    @try_except(reraise=True)
    def fuse_att_quat(self, payload: AVRVIOAttitudeQuaternion) -> None:
        """
        Callback for receiving quaternion att data in NED reference frame
        from vio and publishes into a fusion/att/quat topic.

        AVR doesn't have sophisticated fusion yet, so this just re-routes
        the message onto the fusion topic.
        """
        self.send_message(
            "avr/fusion/attitude/quaternion",
            AVRFusionAttitudeQuaternion(
                w=payload.w, x=payload.x, y=payload.y, z=payload.z
            ),
        )

    @try_except(reraise=True)
    def fuse_att_euler(self, payload: AVRVIOAttitudeEulerRadians) -> None:
        """
        Callback for receiving euler att data in NED reference frame from VIO and
        publishes into a fusion/att/euler topic.

        AVR doesn't have sophisticated fusion yet, so this just re-routes
        the message onto the fusion topic.
        """
        self.send_message(
            "avr/fusion/attitude/euler/radians",
            AVRFusionAttitudeEulerRadians(
                psi=payload.psi, theta=payload.theta, phi=payload.phi
            ),
        )

    @try_except(reraise=True)
    def fuse_att_heading(self, payload: AVRVIOHeading) -> None:
        """
        Callback for receiving heading att data in NED reference frame from VIO and
        publishes into a fusion/att/heading topic.

        AVR doesn't have sophisticated fusion yet, so this just re-routes
        the message onto the fusion topic.
        """
        self.send_message("avr/fusion/heading", AVRFusionHeading(hdg=payload.hdg))

        # if the groundspeed is below the threshold, we lock the course to the heading
        if "avr/fusion/groundspeed" not in self.message_cache:
            logger.debug("Empty groundspeed in fuse att heading")

        elif (
            self.message_cache["avr/fusion/groundspeed"].groundspeed
            < config.COURSE_THRESHOLD
        ):
            self.message_cache["avr/fusion/course"] = AVRFusionCourse(
                course=payload.hdg
            )

    @run_forever(frequency=10)
    @try_except(reraise=False)
    def assemble_hil_gps_message(self) -> None:
        """
        This code takes the pos data from fusion and formats it into a special
        message that is exactly what the FCC needs to generate the hil_gps message
        (with heading)
        """
        if "avr/fusion/position/global" not in self.message_cache:
            logger.debug("Waiting for avr/fusion/position/global to be populated")
            return

        goedetic = self.message_cache["avr/fusion/position/global"]
        lat = int(goedetic.lat * 10000000)  # convert to int32 format
        lon = int(goedetic.lon * 10000000)  # convert to int32 format

        # if lat / lon is 0, that means the ned -> lla conversion hasn't run yet,
        # don't send that data to FCC
        if lat == 0 or lon == 0:
            return

        if "avr/fusion/velocity" not in self.message_cache:
            logger.debug("Waiting for avr/fusion/velocity to be populated")
            return
        elif self.message_cache["avr/fusion/velocity"].Vn is None:
            logger.debug("avr/fusion/velocity message cache is empty")
            return

        crs = 0
        if "avr/fusion/course" in self.message_cache:
            if self.message_cache["avr/fusion/course"].course is not None:
                crs = int(self.message_cache["avr/fusion/course"].course)
        else:
            logger.debug("Waiting for avr/fusion/course message to be populated")
            return

        gs = 0
        if "avr/fusion/groundspeed" in self.message_cache:
            if self.message_cache["avr/fusion/groundspeed"].groundspeed is not None:
                gs = int(self.message_cache["avr/fusion/groundspeed"].groundspeed)
        else:
            logger.debug("avr/fusion/groundspeed message cache is empty")
            return

        if "avr/fusion/heading" in self.message_cache:
            heading = int(self.message_cache["avr/fusion/heading"].hdg * 100)
        else:
            logger.debug("Waiting for avr/fusion/attitude/heading to be populated")
            return

        hil_gps_update = AVRFusionHILGPSMessage(
            time_usec=int(time.time() * 1000000),
            fix_type=int(config.HIL_GPS_CONSTANTS["fix_type"]),  # 3 - 3D fix
            lat=lat,
            lon=lon,
            alt=int(
                self.message_cache["avr/fusion/position/global"].alt * 1000
            ),  # convert m to mm
            eph=int(config.HIL_GPS_CONSTANTS["eph"]),  # cm
            epv=int(config.HIL_GPS_CONSTANTS["epv"]),  # cm
            vel=gs,
            vn=int(self.message_cache["avr/fusion/velocity"].Vn),
            ve=int(self.message_cache["avr/fusion/velocity"].Ve),
            vd=int(self.message_cache["avr/fusion/velocity"].Vd),
            cog=int(crs * 100),
            satellites_visible=int(config.HIL_GPS_CONSTANTS["satellites_visible"]),
            heading=heading,
        )
        self.send_message("avr/fusion/hil_gps/message", hil_gps_update)

    @try_except(reraise=True)
    def on_apriltag_message(self, payload: AVRAprilTagsVehiclePosition) -> None:
        if (
            "avr/fusion/position/local" not in self.message_cache
            or "avr/fusion/heading" not in self.message_cache
        ):
            logger.debug(
                "Waiting for avr/fusion/position/local and avr/fusion/heading to be populated"
            )
            return

        now = time.time()

        # pull ned and heading from cache
        cam_ned = self.message_cache["avr/fusion/position/local"]
        cam_heading = self.message_cache["avr/fusion/heading"].hdg

        at_ned = [payload.x, payload.y, payload.z]

        # compute differences
        n_dist = abs(at_ned[0] - cam_ned.n)
        e_dist = abs(at_ned[1] - cam_ned.e)
        d_dist = abs(at_ned[2] - cam_ned.d)

        norm = np.linalg.norm([n_dist, e_dist, d_dist])

        heading_delta = abs(payload.hdg - cam_heading)
        if heading_delta > 180:
            heading_delta = 360 - heading_delta

        for idx, val in enumerate(at_ned):
            self.deriv[idx] = (val - self.last_pos[idx]) / (now - self.last_apriltag)
            self.last_pos[idx] = val

        deriv_norm = np.linalg.norm(self.deriv)
        if (
            norm > config.POS_DETLA_THRESHOLD
            or abs(heading_delta) > config.HEADING_DELTA_THRESHOLD
        ) and deriv_norm < config.AT_DERIV_THRESHOLD:
            logger.debug(f"Resync Triggered! Delta={norm}")

            if d_dist > config.POS_D_THRESHOLD:
                # don't resync Z if del_d is too great,
                # reject AT readings that are extraineous
                at_ned[2] = cam_ned.d

            self.send_message(
                "avr/vio/resync",
                AVRVIOResync(n=at_ned[0], e=at_ned[1], d=at_ned[2], hdg=payload.hdg),
            )

        self.last_apriltag = now

    def run(self) -> None:
        self.run_non_blocking()
        try:
            self.assemble_hil_gps_message()
        except Exception:
            logger.exception("Issue while assembling hil message")


if __name__ == "__main__":
    fusion = FusionModule()
    fusion.run()
