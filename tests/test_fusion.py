from __future__ import annotations

import time
from typing import TYPE_CHECKING, List, Optional

import pytest
from bell.avr.mqtt.payloads import (
    AvrApriltagsSelectedPayload,
    AvrApriltagsSelectedPos,
    AvrFusionAttitudeEulerPayload,
    AvrFusionAttitudeHeadingPayload,
    AvrFusionAttitudeQuatPayload,
    AvrFusionClimbratePayload,
    AvrFusionCoursePayload,
    AvrFusionGeoPayload,
    AvrFusionGroundspeedPayload,
    AvrFusionHilGpsPayload,
    AvrFusionPositionNedPayload,
    AvrFusionVelocityNedPayload,
    AvrVioHeadingPayload,
    AvrVioOrientationEulPayload,
    AvrVioOrientationQuatPayload,
    AvrVioPositionNedPayload,
    AvrVioResyncPayload,
    AvrVioVelocityNedPayload,
)

if TYPE_CHECKING:
    from src.fusion import FusionModule


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AvrFusionPositionNedPayload(n=0, e=0, d=0),
            AvrFusionGeoPayload(lat=0, lon=0, alt=0),
        ),
        (
            AvrFusionPositionNedPayload(n=1000, e=1000, d=1000),
            AvrFusionGeoPayload(
                lat=9.043709045278911e-05,
                lon=8.983166925499504e-05,
                alt=-9.99998426904285,
            ),
        ),
    ],
)
def test_local_to_geo(
    fusion_module: FusionModule,
    payload: AvrFusionPositionNedPayload,
    expected: AvrFusionGeoPayload,
) -> None:
    fusion_module.local_to_geo(payload)
    fusion_module.send_message.assert_called_once_with("avr/fusion/geo", expected)


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AvrVioPositionNedPayload(n=0, e=0, d=0),
            AvrFusionPositionNedPayload(n=0, e=0, d=0),
        ),
        (
            AvrVioPositionNedPayload(n=1000, e=1000, d=1000),
            AvrFusionPositionNedPayload(n=1000, e=1000, d=1000),
        ),
    ],
)
def test_fuse_pos(
    fusion_module: FusionModule,
    payload: AvrVioPositionNedPayload,
    expected: AvrFusionPositionNedPayload,
) -> None:
    fusion_module.fuse_pos(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/position/ned", expected
    )


@pytest.mark.parametrize(
    "payload, velocity_expected, groundspeed_expected, climbrate_expected",
    [
        (
            AvrVioVelocityNedPayload(n=0, e=0, d=0),
            AvrFusionVelocityNedPayload(Vn=0, Ve=0, Vd=0),
            AvrFusionGroundspeedPayload(groundspeed=0),
            AvrFusionClimbratePayload(climb_rate_fps=0),
        ),
        (
            AvrVioVelocityNedPayload(n=1, e=2, d=3),
            AvrFusionVelocityNedPayload(Vn=1, Ve=2, Vd=3),
            AvrFusionGroundspeedPayload(groundspeed=2.23606797749979),
            AvrFusionClimbratePayload(climb_rate_fps=-590.55),
        ),
    ],
)
def test_fuse_vel(
    fusion_module: FusionModule,
    payload: AvrVioVelocityNedPayload,
    velocity_expected: AvrFusionVelocityNedPayload,
    groundspeed_expected: AvrFusionGroundspeedPayload,
    climbrate_expected: AvrFusionClimbratePayload,
) -> None:
    fusion_module.fuse_vel(payload)
    fusion_module.send_message.assert_any_call(
        "avr/fusion/velocity/ned", velocity_expected
    )
    fusion_module.send_message.assert_any_call(
        "avr/fusion/groundspeed", groundspeed_expected
    )
    fusion_module.send_message.assert_any_call(
        "avr/fusion/climbrate", climbrate_expected
    )
    assert fusion_module.send_message.call_count == 3


@pytest.mark.parametrize(
    "payload, velocity_expected, groundspeed_expected, course_expected, climbrate_expected",
    [
        (
            AvrVioVelocityNedPayload(n=25, e=25, d=0),
            AvrFusionVelocityNedPayload(Vn=25, Ve=25, Vd=0),
            AvrFusionGroundspeedPayload(groundspeed=35.35533905932738),
            AvrFusionCoursePayload(course=45),
            AvrFusionClimbratePayload(climb_rate_fps=0),
        )
    ],
)
def test_fuse_vel_course(
    fusion_module: FusionModule,
    payload: AvrVioVelocityNedPayload,
    velocity_expected: AvrFusionVelocityNedPayload,
    groundspeed_expected: AvrFusionGroundspeedPayload,
    course_expected: AvrFusionCoursePayload,
    climbrate_expected: AvrFusionClimbratePayload,
) -> None:
    """
    At high ground speed, we also publish a course message.
    """

    fusion_module.fuse_vel(payload)
    fusion_module.send_message.assert_any_call(
        "avr/fusion/velocity/ned", velocity_expected
    )
    fusion_module.send_message.assert_any_call(
        "avr/fusion/groundspeed", groundspeed_expected
    )
    fusion_module.send_message.assert_any_call("avr/fusion/course", course_expected)
    fusion_module.send_message.assert_any_call(
        "avr/fusion/climbrate", climbrate_expected
    )
    assert fusion_module.send_message.call_count == 4


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AvrVioOrientationQuatPayload(w=0, x=0, y=0, z=0),
            AvrFusionAttitudeQuatPayload(w=0, x=0, y=0, z=0),
        ),
        (
            AvrVioOrientationQuatPayload(w=1, x=2, y=3, z=4),
            AvrFusionAttitudeQuatPayload(w=1, x=2, y=3, z=4),
        ),
    ],
)
def test_fuse_att_quat(
    fusion_module: FusionModule,
    payload: AvrVioOrientationQuatPayload,
    expected: AvrFusionAttitudeQuatPayload,
) -> None:
    fusion_module.fuse_att_quat(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/attitude/quat", expected
    )


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AvrVioOrientationEulPayload(psi=0, theta=0, phi=0),
            AvrFusionAttitudeEulerPayload(psi=0, theta=0, phi=0),
        ),
        (
            AvrVioOrientationEulPayload(psi=1, theta=2, phi=3),
            AvrFusionAttitudeEulerPayload(psi=1, theta=2, phi=3),
        ),
    ],
)
def test_fuse_att_euler(
    fusion_module: FusionModule,
    payload: AvrVioOrientationEulPayload,
    expected: AvrFusionAttitudeEulerPayload,
) -> None:
    fusion_module.fuse_att_euler(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/attitude/euler", expected
    )


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AvrVioHeadingPayload(degrees=0),
            AvrFusionAttitudeHeadingPayload(heading=0),
        ),
        (
            AvrVioHeadingPayload(degrees=45),
            AvrFusionAttitudeHeadingPayload(heading=45),
        ),
    ],
)
def test_fuse_att_heading_empty(
    fusion_module: FusionModule,
    payload: AvrVioHeadingPayload,
    expected: AvrFusionAttitudeHeadingPayload,
) -> None:
    fusion_module.fuse_att_heading(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/attitude/heading", expected
    )


@pytest.mark.parametrize(
    "payload, expected",
    [
        (
            AvrVioHeadingPayload(degrees=0),
            AvrFusionAttitudeHeadingPayload(heading=0),
        ),
        (
            AvrVioHeadingPayload(degrees=45),
            AvrFusionAttitudeHeadingPayload(heading=45),
        ),
    ],
)
def test_fuse_att_heading_populated(
    fusion_module: FusionModule,
    payload: AvrVioHeadingPayload,
    expected: AvrFusionAttitudeHeadingPayload,
) -> None:
    # preload cache
    fusion_module.message_cache["avr/fusion/groundspeed"] = AvrFusionGroundspeedPayload(
        groundspeed=5
    )

    fusion_module.fuse_att_heading(payload)
    fusion_module.send_message.assert_called_once_with(
        "avr/fusion/attitude/heading", expected
    )

    # make sure item was put in cache
    assert "avr/fusion/course" in fusion_module.message_cache
    assert fusion_module.message_cache["avr/fusion/course"] == AvrFusionCoursePayload(
        course=payload["degrees"]
    )


@pytest.mark.parametrize(
    "geo, velocity, course, groundspeed, heading, expected",
    [
        (
            AvrFusionGeoPayload(lat=10, lon=10, alt=0),
            AvrFusionVelocityNedPayload(Vn=0, Ve=0, Vd=0),
            AvrFusionCoursePayload(course=0),
            AvrFusionGroundspeedPayload(groundspeed=0),
            AvrFusionAttitudeHeadingPayload(heading=0),
            AvrFusionHilGpsPayload(
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
            AvrFusionGeoPayload(lat=25, lon=25, alt=0),
            AvrFusionVelocityNedPayload(Vn=1, Ve=2, Vd=3),
            AvrFusionCoursePayload(course=4),
            AvrFusionGroundspeedPayload(groundspeed=5),
            AvrFusionAttitudeHeadingPayload(heading=6),
            AvrFusionHilGpsPayload(
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
    geo: AvrFusionGeoPayload,
    velocity: AvrFusionVelocityNedPayload,
    course: AvrFusionCoursePayload,
    groundspeed: AvrFusionGroundspeedPayload,
    heading: AvrFusionAttitudeHeadingPayload,
    expected: AvrFusionHilGpsPayload,
) -> None:
    fusion_module.message_cache["avr/fusion/geo"] = geo
    fusion_module.message_cache["avr/fusion/velocity/ned"] = velocity
    fusion_module.message_cache["avr/fusion/course"] = course
    fusion_module.message_cache["avr/fusion/groundspeed"] = groundspeed
    fusion_module.message_cache["avr/fusion/attitude/heading"] = heading

    fusion_module.assemble_hil_gps_message()
    topic, payload = fusion_module.send_message.call_args[0]
    assert topic == "avr/fusion/hil_gps"

    # check everything in the payload except time
    for key, value in payload.items():
        if key != "time_usec":
            print(key)
            assert value == expected[key]


@pytest.mark.parametrize(
    "payload, position, heading, expected_deriv, expected_pos, expected_resync",
    [
        # no resync required
        (
            AvrApriltagsSelectedPayload(
                tag_id=0, pos=AvrApriltagsSelectedPos(n=0, e=0, d=0), heading=0
            ),
            AvrFusionPositionNedPayload(n=0, e=0, d=0),
            AvrFusionAttitudeHeadingPayload(heading=0),
            [0, 0, 0],
            [0, 0, 0],
            None,
        ),
        # test a resync. not too large and not too small of a difference
        (
            AvrApriltagsSelectedPayload(
                tag_id=0, pos=AvrApriltagsSelectedPos(n=3, e=4, d=5), heading=10
            ),
            AvrFusionPositionNedPayload(n=0, e=0, d=0),
            AvrFusionAttitudeHeadingPayload(heading=0),
            [3, 4, 5],
            [3, 4, 5],
            AvrVioResyncPayload(n=3, e=4, d=5, heading=10),
        ),
    ],
)
def test_on_apriltag_message(
    fusion_module: FusionModule,
    payload: AvrApriltagsSelectedPayload,
    position: AvrFusionPositionNedPayload,
    heading: AvrFusionAttitudeHeadingPayload,
    expected_deriv: List[int],
    expected_pos: List[int],
    expected_resync: Optional[AvrVioResyncPayload],
) -> None:
    # we're going back to the future, Marty!
    fusion_module.last_apriltag = time.time() - 1
    fusion_module.message_cache["avr/fusion/position/ned"] = position
    fusion_module.message_cache["avr/fusion/attitude/heading"] = heading

    fusion_module.on_apriltag_message(payload)
    assert fusion_module.deriv == pytest.approx(expected_deriv)
    assert fusion_module.last_pos == pytest.approx(expected_pos)

    if expected_resync:
        fusion_module.send_message.assert_called_once_with(
            "avr/vio/resync", expected_resync
        )
