# Fusion Module

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Build Fusion Module](https://github.com/bellflight/AVR-VMC-Fusion-Module/actions/workflows/build.yml/badge.svg)](https://github.com/bellflight/AVR-VMC-Fusion-Module/actions/workflows/build.yml)

The Fusion module is responsible for fusing multiple data sources to produce
the final fake GPS data that is fed to the FCC. Currently, this only takes data
from the VIO module, but experimental functionality exists to also take data from
the AprilTag module to position itself more accurately in global coordinates.

This is the only module which is pure Python and has no hardware component.
