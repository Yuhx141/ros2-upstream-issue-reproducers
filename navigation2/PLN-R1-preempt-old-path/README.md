# Planner preempt old-path reproducer

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=71 python3 planner_preempt_probe.py control /tmp/planner-control
ROS_DOMAIN_ID=72 python3 planner_preempt_probe.py preempt /tmp/planner-preempt
```

The script launches Planner Server itself. It withholds the StaticLayer map, waits until old and new action handles are accepted, then publishes the free map. Affected builds exit 1 in `preempt` mode because the new successful result ends at the old goal.
