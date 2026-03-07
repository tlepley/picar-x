# Original SunFounder Examples

This directory keeps the original SunFounder PI-CAR-X example scripts from the 2.1.x codebase.

These files are preserved on purpose. They are useful if you want to run PI-CAR-X in its original non-ROS mode, directly from Python on the car itself.

## When To Use These Examples

Use the scripts in this directory if you want:

- the original SunFounder workflow
- direct hardware access without ROS
- quick hardware validation on the PI-CAR-X board
- comparison between original behavior and the ROS-based workflow

## Basic Usage

From the repository root:

```bash
cd ~/git/picar-x
python3 -m pip install -e .
python3 example/4.avoiding_obstacles.py
```

Another example:

```bash
cd ~/git/picar-x
python3 example/6.line_tracking.py
```

## Notes

- These scripts access PI-CAR-X hardware directly through the original `picarx` Python API.
- They are intended to run on the PI-CAR-X board itself.
- They do not use ROS topics, ROS nodes, or the ROS calibration workflow.

## Relationship With The ROS Version

This repository also provides a ROS 2 integration layer.

- Original examples in this directory: direct Python / non-ROS mode
- ROS version: remote/local split under [`ros/`](../ros)

If you want to use the ROS version, see [`ros/README.md`](../ros/README.md).

## Upstream Documentation

The original SunFounder documentation explains how these examples are intended to be installed and used.

- Picar-X documentation: <https://docs.sunfounder.com/projects/picar-x-v20/en/latest/>
- Robot Hat documentation: <https://docs.sunfounder.com/projects/robot-hat-v4/en/latest/>
- SunFounder forum: <https://forum.sunfounder.com/>
- SunFounder website: <https://www.sunfounder.com/>
