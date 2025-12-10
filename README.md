# LAB3: Eater vs. Killer - Multi-Agent Turtle Simulation

## Quick Setup

```bash
# Clone the repository
git clone https://github.com/BhumipatNgamphueak/FRA502-LAB-6643.git -b LAB3 
cd FRA502-LAB-6643

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Launch

```bash
ros2 launch lab3 lab3_bringup.launch.py
```

## Examples

**Namespaces**: Eater = `Peemai`, Killer = `killer`

> **Note**: All service calls below use the default namespaces. If you change the namespace in your launch file, replace `/Peemai/` or `/killer/` with your custom namespace in all commands.

### Configure Controller Gains

```bash
# Eater controller
ros2 service call /Peemai/set_Param controller_interfaces/srv/SetParam \
  "{kp_linear: {data: 2.5}, kp_angular: {data: 12.0}}"

# Killer controller
ros2 service call /killer/set_Param controller_interfaces/srv/SetParam \
  "{kp_linear: {data: 3.0}, kp_angular: {data: 15.0}}"
```

### Set Maximum Pizza Count

```bash
# Increase max pizza (returns "success")
ros2 service call /Peemai/set_max_pizza controller_interfaces/srv/SetMaxpizza \
  "{max_pizza: {data: 10}}"

# Decrease max pizza (returns "failed")
ros2 service call /Peemai/set_max_pizza controller_interfaces/srv/SetMaxpizza \
  "{max_pizza: {data: 3}}"
```

### Parameters

```bash
# Get sampling frequency
ros2 param get /Peemai/Eater sampling_frequency

ros2 param get /killer/Killer sampling_frequency

# Set sampling frequency at runtime
ros2 param set /Peemai/Eater sampling_frequency 150.0
```
