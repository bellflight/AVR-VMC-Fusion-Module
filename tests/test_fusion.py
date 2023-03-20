from __future__ import annotations

import time
from typing import TYPE_CHECKING, List, Optional

import pytest
from bell.avr.mqtt.payloads import (
    AVRFusionPositionGlobal,
    AVRFusionPositionLocal,
    AVRVIOPositionLocal,
    AVRVIOVelocity,
    AVRFusionVelocity,
    AVRFusionGroundspeed,
    AVRFusionClimbRate,
    AVRFusionCourse,
    AVRVIOAttitudeQuaternion,
    AVRFusionAttitudeQuaternion,
    AVRVIOAttitudeEulerRadians,
    AVRFusionAttitudeEulerRadians,
    AVRVIOHeading,
    AVRFusionHeading,
    AVRFusionHILGPSMessage,
    AVRVIOResync,
    AVRAprilTagsVehiclePosition,
)

if TYPE_CHECKING:
    from src.fusion import FusionModule


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AVRFusionPositionLocal(n=0, e=0, d=0),
            AVRFusionPositionGlobal(lat=0, lon=0, alt=0),
        ),
        (
            AVRFusionPositionLocal(n=1000, e=1000, d=1000),
            AVRFusionPositionGlobal(
                lat=9.043709045278911e-05,
                lon=8.983166925499504e-05,
                alt=-9.99998426904285,
            ),
        ),
    ],
)
def test_local_to_geo(
    fusion_module: FusionModule,
    payload: AVRFusionPositionLocal,
    expected: AVRFusionPositionGlobal,
) -> None:
    fusion_module.local_to_geo(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/position/global", expected
    )


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AVRVIOPositionLocal(n=0, e=0, d=0),
            AVRFusionPositionLocal(n=0, e=0, d=0),
        ),
        (
            AVRVIOPositionLocal(n=1000, e=1000, d=1000),
            AVRFusionPositionLocal(n=1000, e=1000, d=1000),
        ),
    ],
)
def test_fuse_pos(
    fusion_module: FusionModule,
    payload: AVRVIOPositionLocal,
    expected: AVRFusionPositionLocal,
) -> None:
    fusion_module.fuse_pos(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/position/local", expected
    )


@pytest.mark.parametrize(
    "payload, velocity_expected, groundspeed_expected, climbrate_expected",
    [
        (
            AVRVIOVelocity(Vn=0, Ve=0, Vd=0),
            AVRFusionVelocity(Vn=0, Ve=0, Vd=0),
            AVRFusionGroundspeed(groundspeed=0),
            AVRFusionClimbRate(climb_rate=0),
        ),
        (
            AVRVIOVelocity(Vn=1, Ve=2, Vd=3),
            AVRFusionVelocity(Vn=1, Ve=2, Vd=3),
            AVRFusionGroundspeed(groundspeed=2.23606797749979),
            AVRFusionClimbRate(climb_rate=-590.55),
        ),
    ],
)
def test_fuse_vel(
    fusion_module: FusionModule,
    payload: AVRVIOVelocity,
    velocity_expected: AVRFusionVelocity,
    groundspeed_expected: AVRFusionGroundspeed,
    climbrate_expected: AVRFusionClimbRate,
) -> None:
    fusion_module.fuse_vel(payload)
    fusion_module.send_message.assert_any_call("avr/fusion/velocity", velocity_expected)
    fusion_module.send_message.assert_any_call(
        "avr/fusion/groundspeed", groundspeed_expected
    )
    fusion_module.send_message.assert_any_call(
        "avr/fusion/climb_rate", climbrate_expected
    )
    assert fusion_module.send_message.call_count == 3


@pytest.mark.parametrize(
    "payload, velocity_expected, groundspeed_expected, course_expected, climbrate_expected",
    [
        (
            AVRVIOVelocity(Vn=25, Ve=25, Vd=0),
            AVRFusionVelocity(Vn=25, Ve=25, Vd=0),
            AVRFusionGroundspeed(groundspeed=35.35533905932738),
            AVRFusionCourse(course=45),
            AVRFusionClimbRate(climb_rate=0),
        )
    ],
)
def test_fuse_vel_course(
    fusion_module: FusionModule,
    payload: AVRVIOVelocity,
    velocity_expected: AVRFusionVelocity,
    groundspeed_expected: AVRFusionGroundspeed,
    course_expected: AVRFusionCourse,
    climbrate_expected: AVRFusionClimbRate,
) -> None:
    """
    At high ground speed, we also publish a course message.
    """

    fusion_module.fuse_vel(payload)
    fusion_module.send_message.assert_any_call("avr/fusion/velocity", velocity_expected)
    fusion_module.send_message.assert_any_call(
        "avr/fusion/groundspeed", groundspeed_expected
    )
    fusion_module.send_message.assert_any_call("avr/fusion/course", course_expected)
    fusion_module.send_message.assert_any_call(
        "avr/fusion/climb_rate", climbrate_expected
    )
    assert fusion_module.send_message.call_count == 4


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AVRVIOAttitudeQuaternion(w=0, x=0, y=0, z=0),
            AVRFusionAttitudeQuaternion(w=0, x=0, y=0, z=0),
        ),
        (
            AVRVIOAttitudeQuaternion(w=0.2, x=0.4, y=0.6, z=0.8),
            AVRFusionAttitudeQuaternion(w=0.2, x=0.4, y=0.6, z=0.8),
        ),
    ],
)
def test_fuse_att_quat(
    fusion_module: FusionModule,
    payload: AVRVIOAttitudeQuaternion,
    expected: AVRFusionAttitudeQuaternion,
) -> None:
    fusion_module.fuse_att_quat(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/attitude/quaternion", expected
    )


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AVRVIOAttitudeEulerRadians(psi=0, theta=0, phi=0),
            AVRFusionAttitudeEulerRadians(psi=0, theta=0, phi=0),
        ),
        (
            AVRVIOAttitudeEulerRadians(psi=1, theta=2, phi=3),
            AVRFusionAttitudeEulerRadians(psi=1, theta=2, phi=3),
        ),
    ],
)
def test_fuse_att_euler(
    fusion_module: FusionModule,
    payload: AVRVIOAttitudeEulerRadians,
    expected: AVRFusionAttitudeEulerRadians,
) -> None:
    fusion_module.fuse_att_euler(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/attitude/euler/radians", expected
    )


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AVRVIOHeading(hdg=0),
            AVRFusionHeading(hdg=0),
        ),
        (
            AVRVIOHeading(hdg=45),
            AVRFusionHeading(hdg=45),
        ),
    ],
)
def test_fuse_att_heading_empty(
    fusion_module: FusionModule,
    payload: AVRVIOHeading,
    expected: AVRFusionHeading,
) -> None:
    fusion_module.fuse_att_heading(payload)
    fusion_module.send_message.assert_called_once_with("avr/fusion/heading", expected)


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AVRVIOHeading(hdg=0),
            AVRFusionHeading(hdg=0),
        ),
        (
            AVRVIOHeading(hdg=45),
            AVRFusionHeading(hdg=45),
        ),
    ],
)
def test_fuse_att_heading_populated(
    fusion_module: FusionModule,
    payload: AVRVIOHeading,
    expected: AVRFusionHeading,
) -> None:
    # preload cache
    fusion_module.message_cache["avr/fusion/groundspeed"] = AVRFusionGroundspeed(
        groundspeed=5
    )

    fusion_module.fuse_att_heading(payload)
    fusion_module.send_message.assert_called_once_with("avr/fusion/heading", expected)

    # make sure item was put in cache
    assert "avr/fusion/course" in fusion_module.message_cache
    assert fusion_module.message_cache["avr/fusion/course"] == AVRFusionCourse(
        course=payload.hdg
    )


@pytest.mark.parametrize(
    "geo, velocity, course, groundspeed, heading, expected",
    [
        (
            AVRFusionPositionGlobal(lat=10, lon=10, alt=0),
            AVRFusionVelocity(Vn=0, Ve=0, Vd=0),
            AVRFusionCourse(course=0),
            AVRFusionGroundspeed(groundspeed=0),
            AVRFusionHeading(hdg=0),
            AVRFusionHILGPSMessage(
                time_usec=0,
                fix_type=3,
                lat=100000000,
                lon=100000000,
                alt=0,
                eph=20,
                epv=5,
                vel=0,
                vn=0,
                ve=0,
                vd=0,
                cog=0,
                satellites_visible=13,
                heading=0,
            ),
        ),
        (
            AVRFusionPositionGlobal(lat=25, lon=25, alt=0),
            AVRFusionVelocity(Vn=1, Ve=2, Vd=3),
            AVRFusionCourse(course=4),
            AVRFusionGroundspeed(groundspeed=5),
            AVRFusionHeading(hdg=6),
            AVRFusionHILGPSMessage(
                time_usec=0,
                fix_type=3,
                lat=250000000,
                lon=250000000,
                alt=0,
                eph=20,
                epv=5,
                vel=5,
                vn=1,
                ve=2,
                vd=3,
                cog=400,
                satellites_visible=13,
                heading=600,
            ),
        ),
    ],
)
def test_assemble_hil_gps_message(
    fusion_module: FusionModule,
    geo: AVRFusionPositionGlobal,
    velocity: AVRFusionVelocity,
    course: AVRFusionCourse,
    groundspeed: AVRFusionGroundspeed,
    heading: AVRFusionHeading,
    expected: AVRFusionHILGPSMessage,
) -> None:
    fusion_module.message_cache["avr/fusion/position/global"] = geo
    fusion_module.message_cache["avr/fusion/velocity"] = velocity
    fusion_module.message_cache["avr/fusion/course"] = course
    fusion_module.message_cache["avr/fusion/groundspeed"] = groundspeed
    fusion_module.message_cache["avr/fusion/heading"] = heading

    fusion_module.assemble_hil_gps_message()
    topic, payload = fusion_module.send_message.call_args[0]
    assert topic == "avr/fusion/hil_gps/message"

    # check everything in the payload except time
    fields = list(payload.__dict__.keys())
    fields.remove("time_usec")

    for field in fields:
        assert getattr(payload, field) == getattr(expected, field)


@pytest.mark.parametrize(
    "payload, position, heading, expected_deriv, expected_pos, expected_resync",
    [
        # no resync required
        (
            AVRAprilTagsVehiclePosition(tag_id=0, x=0, y=0, z=0, hdg=0),
            AVRFusionPositionLocal(n=0, e=0, d=0),
            AVRFusionHeading(hdg=0),
            [0, 0, 0],
            [0, 0, 0],
            None,
        ),
        # test a resync. not too large and not too small of a difference
        (
            AVRAprilTagsVehiclePosition(tag_id=0, x=3, y=4, z=5, hdg=10),
            AVRFusionPositionLocal(n=0, e=0, d=0),
            AVRFusionHeading(hdg=0),
            [3, 4, 5],
            [3, 4, 5],
            AVRVIOResync(n=3, e=4, d=5, hdg=10),
        ),
    ],
)
def test_on_apriltag_message(
    fusion_module: FusionModule,
    payload: AVRAprilTagsVehiclePosition,
    position: AVRFusionPositionLocal,
    heading: AVRFusionHeading,
    expected_deriv: List[int],
    expected_pos: List[int],
    expected_resync: Optional[AVRVIOResync],
) -> None:
    # we're going back to the future, Marty!
    fusion_module.last_apriltag = time.time() - 1
    fusion_module.message_cache["avr/fusion/position/local"] = position
    fusion_module.message_cache["avr/fusion/heading"] = heading

    fusion_module.on_apriltag_message(payload)
    assert fusion_module.deriv == pytest.approx(expected_deriv, rel=1e-3)
    assert fusion_module.last_pos == pytest.approx(expected_pos, rel=1e-3)

    if expected_resync:
        fusion_module.send_message.assert_called_once_with(
            "avr/vio/resync", expected_resync
        )
