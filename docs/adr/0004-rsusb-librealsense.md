# ADR-0004: librealsense from source with the RSUSB backend

Status: accepted · Date: 2026-07-29

## Context

The D455 IMU is the yaw reference the EKF depends on. The apt
`ros-humble-librealsense2` (arm64) cannot read the IMU on this Pi: it wants a
kernel HID-sensor path that does not exist (the Pi kernel ships no
`hid-sensor-*` modules), so the camera stays on generic `usbhid`, no IIO device
appears, and the stack silently produces no IMU.

## Decision

The Dockerfile builds librealsense **v2.57.7 from source with
`FORCE_RSUSB_BACKEND=ON`** (raw libusb), and realsense-ros 4.57.7 to match
(wrapper `4.X.Y` ↔ lib `2.X.Y`; bump both together). The `cmake` invocation is
known-good — every failure so far has been a missing apt package
(`libudev-dev`, `python3-dev`), fixed in the system-deps layer.

## Consequences

- ~13 min of the image build is this layer; do not "simplify" it to apt.
- It is the one source build outside the `$OVERLAY` convention (installs into
  `/opt/ros/humble`, arch-locked libdir). Detail: CLAUDE.md "D455 IMU".
