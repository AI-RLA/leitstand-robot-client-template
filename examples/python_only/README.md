# Python-only example

A robot with no ROS. Three files:

- `my_navigation.py` implements `Navigation`: replace `drive_leg` with what moves your machine.
- `my_pose_source.py` implements `PoseSource`: replace `read_position` with your GNSS receiver.
- `main.py` loads `robot.yaml`, opens the Zenoh session and hands both to `LeitstandClient`.

Run it, from this directory, with the library and the contract installed:

    pip install ../../../leitstand-robot-contract     # the checkout next to this repo, at v0.4.0
    pip install ../../leitstand_client
    python main.py robot.yaml

The robot appears in the fleet view as `python_example` within a few seconds and accepts
navigation missions. `main.py` ignores the `navigation:` key because it constructs its own.
