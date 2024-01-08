# TUBAF_Robotik_23_24
Repository for the winter semester 23/24 robotics course at the TU Freiberg 

## Projects
+ [Timing_Tubaf](#timing_tubaf)

## Timing_Tubaf
Minimalist example demonstrating a C++ publisher node, publishing an integer on topic "number"
and a Python node subscribing to "number" and publishing the temporal difference between 
updates to "number" on topic "diff".

### Build
In the root directory call:

```colcon build --packages-select timing_tubaf```

### Run
In the root directory call:

```
source install/setup.bash
ros2 run timing_tubaf my_publisher
```

In another terminal, again in the root directory, call:

```
source install/setup.bash
ros2 run timing_tubaf my_subscriber.py
```

The order in which you execute these should not matter.
