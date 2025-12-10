# LAB2: Eater vs. Killer - Multi-Agent Turtle Simulation

## Quick Setup

```bash
# Clone the repository
git clone https://github.com/BhumipatNgamphueak/FRA502-LAB-6643.git -b LAB2
cd FRA502-LAB-6643

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Add to bashrc
```bash
echo "source ~/FRA502-LAB-6643/install/setup.bash" >> ~/.bashrc
```

## Running the Simulation

### Option 1: Individual Run

Run each node separately in different terminals:

**Terminal 1: Start TurtleSim Plus**
```bash
ros2 run turtlesim_plus turtlesim_plus_node.py
```

**Terminal 2: Start TurtleSim Pose**
```bash
ros2 run lab2 turtlesim_pose.py
```

**Terminal 3: Start Eater Node**
```bash
ros2 run lab2 eater.py
```

**Terminal 4: Start Killer Node**
```bash
ros2 run lab2 killer.py
```

**Terminal 5: RViz Visualization**
```bash
rviz2 -d ~/Lab2_ws/src/lab2.rviz
```

### Option 2: Launch File

Launch all nodes together using the launch file:

**Terminal 1: Launch main nodes**
```bash
ros2 launch lab2 simple_launch.py
```

**Terminal 2: Start TurtleSim Pose (required)**
```bash
ros2 run lab2 turtlesim_pose.py
```

**Terminal 3: RViz Visualization (optional)**
```bash
rviz2 -d ~/Lab2_ws/src/lab2.rviz
```

## Configuration

### Set Maximum Pizza

```bash
ros2 topic pub /set_max_pizza std_msgs/msg/Int16 "data: 10" 
```

**Default:** 5 pizzas

### Killer Turtle Spawn

**If killer turtle doesn't appear, manually spawn it using:**

```bash
ros2 service call /spawn_turtle turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.2, name: 'turtle2'}"
```


