from __future__ import annotations

import sys
from typing import TYPE_CHECKING

import pytest
from bell.avr.utils.testing import dont_run_forever
from pytest_mock.plugin import MockerFixture

if TYPE_CHECKING:
    from src.fusion import FusionModule


@pytest.fixture
def fusion_module(mocker: MockerFixture) -> FusionModule:
    # patch the run_forever decorator
    mocker.patch("bell.avr.utils.decorators.run_forever", dont_run_forever)

    # patch the send message function
    sys.path.append("src")
    mocker.patch("src.fusion.FusionModule.send_message")

    # overwrite default lat/lon for testing
    mocker.patch("config.ORIGIN", (0.0, 0.0, 0.0))
    # also for testing, make these constant
    mocker.patch(
        "config.HIL_GPS_CONSTANTS",
        {
            "fix_type": 3,
            "eph": 20,
            "epv": 5,
            "satellites_visible": 13,
        },
    )
    mocker.patch("config.COURSE_THRESHOLD", 10)
    mocker.patch("config.POS_DETLA_THRESHOLD", 10)
    mocker.patch("config.POS_D_THRESHOLD", 30)
    mocker.patch("config.HEADING_DELTA_THRESHOLD", 5)
    mocker.patch("config.AT_DERIV_THRESHOLD", 10)

    # create module object
    from src.fusion import FusionModule

    return FusionModule()
