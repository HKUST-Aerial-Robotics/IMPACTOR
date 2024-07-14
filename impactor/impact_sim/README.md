# impact_sim
## Introduction
Quadrotor with cable-suspended payload simulation based on Drake.

## Installation
- Install ROS.
- Create a virtual environment and install Drake:
```
python -m venv impact_sim
impact_sim/bin/pip install --upgrade pip
impact_sim/bin/pip install drake
```
- For Ubuntu 20.04, install these additional libraries:
```
sudo apt-get install --no-install-recommends \
  libpython3.8 libx11-6 libsm6 libxt6 libglib2.0-0
```
- Active the virtual environment:
```
source impact_sim/bin/activate
```

## Run
- Active the virtual environment.
- Create 3 terminators(if use remote controller).
- In the first terminator, run:
```
python -m pydrake.visualization.meldis -w
```
- In the seconed terminator, run:
```
python payload_sim/scripts/client.py
```
- In the last terminator, run:
```
python payload_sim/scripts/main.py
```
- (Optional) To run the local controller version, you only need to change the parameter `use_remote_control` in `impact_sim/config/config.yaml`. And no need to run the `client.py`