"""Pure domain logic for Scout — stdlib + numpy only, NO ROS.

Every module here must import cleanly on a plain Python install (the dev Mac,
CI) so its tests run without a ROS environment — no rclpy, no tf2_ros, no
message types, no yaml. That rule is enforced by test/test_core_purity.py.

The node glue (rclpy nodes, tf2 lookups, message (de)serialization) lives in
the node modules and scout/node_util.py and calls into here. Keeping the
algorithms here means the real logic — geometry, the under-lidar grid, the
battery curve, coverage planning — is testable through a plain function
interface instead of only by standing up a Node.
"""
