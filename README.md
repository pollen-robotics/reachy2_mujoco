# Reachy2 fake mujoco sdk server

## Installation

```bash
pip install -e .
```

## Usage

Run the server :

```bash
$ reachy2-mujoco
```

Then you can use the ReachySDK as usual, just import it from `reachy2_mujoco` :

```python
from reachy2_mujoco import ReachySDK

reachy = ReachySDK("localhost")

reachy.mobile_base.goto(-1.0, 1.0, -np.pi / 2)

reachy.r_arm.shoulder_pitch.goal_position = np.pi/2

reachy.send_goal_positions()
```

## TODO
- make cameras work
- Make it a package that can be easilly installed
  - just pop a reachy mujoco server
  - then you can import the equivalent of `from reachy2_sdk import ReachySDK` from anywhere etc -> should be `from reachy2_mujoco import ReachySDK`
  - Should be as transparent as possible
- Improve mobile base behavior