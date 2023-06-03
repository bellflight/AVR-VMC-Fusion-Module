# Fusion Module

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Build Fusion Module](https://github.com/bellflight/AVR-VMC-Fusion-Module/actions/workflows/build.yml/badge.svg)](https://github.com/bellflight/AVR-VMC-Fusion-Module/actions/workflows/build.yml)

The Fusion module is responsible for fusing multiple data sources to produce
the final fake GPS data that is fed to the FCC. Currently, this only takes data
from the VIO module, but experimental functionality exists to also take data from
the AprilTag module to position itself more accurately in global coordinates.

This is the only module which is pure Python and has no hardware component.

## Development

It's assumed you have a version of Python installed from
[python.org](https://python.org) that is the same or newer as
defined in the [`Dockerfile`](Dockerfile).

First, install [Poetry](https://python-poetry.org/) and
[VS Code Task Runner](https://pypi.org/project/vscode-task-runner/):

```bash
python -m pip install pipx --upgrade
pipx ensurepath
pipx install poetry
pipx install vscode-task-runner
# (Optionally) Add pre-commit plugin
poetry self add poetry-pre-commit-plugin
```

Now, you can clone the repo and install dependencies:

```bash
git clone https://github.com/bellflight/AVR-VMC-Fusion-Module
cd AVR-VMC-Fusion-Module
vtr install
```

Run

```bash
poetry shell
```

to activate the virtual environment.
