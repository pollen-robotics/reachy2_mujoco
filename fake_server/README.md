`pip install -r requirements.txt`

In separate terminals :

run `mujoco_server.py`

run `client.py`

# TODO
- make cameras work
- Make it a package that can be easilly installed
  - just pop a reachy mujoco server
  - then you can import the equivalent of `from reachy2_sdk import ReachySDK` from anywhere etc -> should be `from reachy2_mujoco import ReachySDK`
  - Should be as transparent as possible