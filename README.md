# Maliput-Webots Worlds

https://github.com/user-attachments/assets/9a406095-172e-45ff-b61e-0d7fb8b937f0

## Description

This repository contains a Webots project with:
 - Wolds:
   - Two simple road networks are provided
 - Controller:
   - Pure pursuit controller that uses [maliput](https://maliput.readthedocs.io/en/latest/) for representing the road network.


## Requirements

- maliput (Foxy):
  - [Maliput Binary installation](https://maliput.readthedocs.io/en/latest/installation.html#id6)
    - It is recommended to install the `maliput-full` package.
- Webots
  - [Webots installation](https://cyberbotics.com/doc/guide/installing-webots)

## Try it!

source underlying binaries

```
source /opt/ros/foxy/setup.bash
```

Run webots and use the user interface to load your desired world.

```
webots
```

or directly run webots using the desired world.

```
webots worlds/loop_road_pedestrian_crosswalk_single_agent.wbt
```

## Available worlds
 - loop_road_pedestrian_crosswalk_single_agent.wbt
 - loop_road_pedestrian_crosswalk_multi_agent.wbt
 - circuit_single_agent.wbt
 - circuit_multi_agent.wbt
