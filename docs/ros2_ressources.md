# ROS2 ressources

## Parallelization
This blog describe well how to parallelize ros2 callbacks
[Deadlocks in rclpy and how to prevent them with use of callback groups](https://karelics.fi/blog/2022/04/21/deadlocks-in-rclpy/)
This are good praticies to control executions :
```bash
 - Register callbacks accessing critical non-thread-safe resources in the same MutuallyExclusiveCallbackGroup (or protect the resources by locks manually).
 - If you have a callback whose execution instances need to be able to overlap with each other, register it to a ReentrantCallbackGroup.
 - If you have callbacks that require to be potentially executed in parallel to one another, register them to
    - a ReentrantCallbackGroup, or
    - different MutuallyExclusiveCallbackGroups (this option is good if you want the callbacks to not overlap themselves, or also need or want thread safety with respect to some other callbacks).
```
It also explain how to make ROS2 synchronous callbacks withhout issues.
