# Reachy2 fake mujoco sdk server

You can interact with Reachy2 in mujoco through Reachy2's sdk! 

For now this is a re-implementation of the sdk, far from feature complete, but with enough basic fonctionnality to perform complex tasks.

Features :
- joint space control
- cartesian control with goto 
- mobile base control
- access to the cameras

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
# target_pose = <...> # 4x4 pose matrix
# reachy.r_arm.goto(target_pose)

reachy.send_goal_positions()
```

## TODO
- Match mobile base behavior with real one
- Better tune the actuators to match real behavior (ideally, perform system identification)
- Improve performance