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
- match mobile base behavior with real one