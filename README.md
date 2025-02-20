# Reachy2 fake mujoco sdk server

You can interact with Reachy2 in mujoco through Reachy2's (fake) sdk! 

![Capture d’écran du 2025-02-20 10-12-34](https://github.com/user-attachments/assets/d363d60a-0881-483e-8240-3dab9250eee8)


For now this is a re-implementation of the sdk, far from feature complete, but with enough basic fonctionnality to perform complex tasks.

Features :
- joint space control
- 6D end effector space control with goto and Reachy2's [symbolic ik](https://github.com/pollen-robotics/reachy2_symbolic_ik)
- mobile base control
- access to the cameras (rgb and depth)

> This is an early prototype, expect bugs :)

## Installation

> Using a virtual environment is advised

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

## Demos

https://github.com/user-attachments/assets/71dba964-e955-49fd-96fd-f75069f70bb4

https://github.com/user-attachments/assets/5f57de88-49db-4243-ae3a-73185efffbfc

https://github.com/user-attachments/assets/60fa3aa7-9cdc-45d8-9a1d-d74ed3cb4301



## TODO
- Match mobile base behavior with real one
- Better tune the actuators to match real behavior (ideally, perform system identification)
- Check that depth is metric
- Improve performance
