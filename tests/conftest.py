from __future__ import annotations

from functools import wraps
from typing import TYPE_CHECKING, Any, Callable

import pytest
from pytest_mock.plugin import MockerFixture

if TYPE_CHECKING:
    from src.fusion import FusionModule


def dont_run_forever(*args, **kwargs) -> Callable:
    def decorator(f: Callable) -> Callable:
        @wraps(f)
        def wrapper(*args, **kwargs) -> Any:
            return f(*args, **kwargs)

        return wrapper

    return decorator


@pytest.fixture
def fusion_module(mocker: MockerFixture) -> FusionModule:
    # patch the run_forever decorator
    mocker.patch("bell.avr.utils.decorators.run_forever", dont_run_forever)

    # patch the send message function
    mocker.patch("src.fusion.FusionModule.send_message")

    # create module object
    from src.fusion import FusionModule

    module = FusionModule()

    # overwrite default lat/lon for testing
    module.ORIGIN = (0.0, 0.0, 0.0)

    return module
