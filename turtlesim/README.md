# Tutorial - Getting Started with ROS MCP Server and Turtlesim

Turtlesim is a lightweight simulator that demonstrates the fundamental concepts of ROS at the most basic level. It's ideal for understanding how the MCP server can interact with ROS systems before moving on to more complex robots.

## What Todo

By the end of this tutorial, we'll be able to:
- Launch Turtlesim on the ROS system
- Explore ROS topics and services
- Control the turtle using natural language commands through the MCP server
- Understand the basic concepts of ROS-MCP integration

## Prerequisites

Before starting this tutorial, make sure you have:

✅ **Any version of ROS installed** (ROS1 Noetic, ROS2 Humble, or ROS2 Jazzy)  
✅ **Basic familiarity with terminal/command line**  
✅ **The ROS MCP Server installed** (see [Installation Guide](../../docs/installation.md) for setup instructions)

## Step 1: Launch Turtlesim

First, let's get Turtlesim running. The exact command depends on your ROS version:

<details>
<summary><strong>ROS1 (e.g., Noetic)</strong></summary>

### Launch Turtlesim

1. **Source your ROS environment:**
   ```bash
   source /opt/ros/noetic/setup.bash # or /opt/ros/<ros_distro>/setup.bash
   ```

2. **Launch Turtlesim:**
   ```bash
   rosrun turtlesim turtlesim_node
   ```

You should see a window appear with a turtle in the center of a blue background.

### Troubleshooting ROS1
- If you get "command not found", make sure you've sourced the ROS environment
- If the window doesn't appear, check your display settings (especially on WSL or remote connections)

</details>

<details>
<summary><strong>ROS2 (e.g., Humble, Jazzy)</strong></summary>

### Launch Turtlesim

1. **Source your ROS2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash  # or /opt/ros/jazzy/setup.bash
   ```

2. **Launch Turtlesim:**
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

You should see a window appear with a turtle in the center of a blue background.

### Troubleshooting ROS2
- If you get "command not found", make sure you've sourced the ROS2 environment
- If the window doesn't appear, check your display settings (especially on WSL or remote connections)

</details>

## Step 2: Explore ROS Topics and Services

Now that Turtlesim is running, let's explore what's available in the ROS system. Open a new terminal and source your ROS environment, then try these commands:

### List Available Topics

<details>
<summary><strong>ROS1 (e.g., Noetic)</strong></summary>

```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# List all topics
rostopic list

# Monitor turtle position
rostopic echo /turtle1/pose

# Monitor velocity commands
rostopic echo /turtle1/cmd_vel
```

</details>

<details>
<summary><strong>ROS2 (e.g., Humble, Jazzy)</strong></summary>

```bash
# Source ROS2 environment (adjust for your version)
source /opt/ros/humble/setup.bash  # or /opt/ros/jazzy/setup.bash

# List all topics
ros2 topic list

# Monitor turtle position
ros2 topic echo /turtle1/pose

# Monitor velocity commands
ros2 topic echo /turtle1/cmd_vel
```

</details>

### List Available Services

<details>
<summary><strong>ROS1 (e.g., Noetic)</strong></summary>

```bash
# List all services
rosservice list

# Get information about a specific service
rosservice info /turtle1/set_pen
```

</details>

<details>
<summary><strong>ROS2 (e.g., Humble, Jazzy)</strong></summary>

```bash
# List all services
ros2 service list

# Get information about a specific service
ros2 service type /turtle1/set_pen
```

</details>

### Understanding Topics and Services

- **Topics** are like radio stations - nodes can publish data to topics and subscribe to receive data
- **Services** are like function calls - you can request a specific action and get a response
- The turtle's position is published on `/turtle1/pose`
- Movement commands are sent via `/turtle1/cmd_vel`
- Services like `/turtle1/set_pen` can change the turtle's drawing properties

## Step 3: Install and Configure the MCP Server

If you haven't already set up the ROS MCP Server, follow the detailed [Installation Guide](../../docs/installation.md). The MCP server can run on:

- **Same machine** as your ROS system (simplest setup)
- **Different machine** on the same local network (for remote control)

The installation guide covers:
- Installing the MCP server
- Configuring your language model client (Claude Desktop, etc.)
- Setting up rosbridge for communication

## Step 4: Hands-on Exploration with MCP Server

Now for the fun part! Once your MCP server is connected, you can control the turtle using natural language. Here are some commands to try:

### Basic Movement Commands

Try these natural language commands with your AI assistant:

```
Move the turtle forward
```

```
Turn the turtle left
```

```
Make the turtle go backward
```

```
Stop the turtle
```

### Information Queries

Ask your AI assistant about the robot's state:

```
Tell me about this robot.
```


```
What topics and services are available on the robot?
```

```
What is the turtle's current position?
```



### Setup Commands


```
Reset the turtle to the center
```

```
Change the turtle's pen color to red
```

```
Spawn a new turtle
```

```
Clear the background
```

### Advanced Commands

Try more complex behaviors:

```
Draw a square with the turtle
```

```
Move the turtle to position (5, 5)
```

```
Make the turtle follow a circular path
```

```
Draw a spiral pattern
```

### Pro Tips

- **Be specific**: Instead of "move", try "move forward at 2 m/s"
- **Ask questions**: "What can this robot do?" or "How do I make the turtle draw?"
- **Experiment**: Try combining commands like "draw a square, then change the pen color to green"
- **Monitor**: Use `rostopic echo` in a separate terminal to see the commands being sent

## Troubleshooting

### Common Issues

<details>
<summary><strong>MCP Server Connection Issues</strong></summary>

**Problem**: AI assistant can't connect to the robot

**Solutions**:
- Verify rosbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check if MCP server is running and connected
- Ensure firewall allows WebSocket connections (port 9090)
- For remote connections, verify the robot's IP address

</details>

<details>
<summary><strong>ROS Environment Issues</strong></summary>

**Problem**: "command not found" errors

**Solutions**:
- Always source your ROS environment: `source /opt/ros/[version]/setup.bash`
- Add sourcing to your `.bashrc` for automatic setup
- Verify ROS installation with `rosversion -d` (ROS1) or `ros2 doctor` (ROS2)

</details>

<details>
<summary><strong>Display Issues</strong></summary>

**Problem**: Turtlesim window doesn't appear

**Solutions**:
- **WSL users**: Install X11 forwarding: `sudo apt install x11-apps`
- **Remote connections**: Use X11 forwarding: `ssh -X username@hostname`

</details>

<details>
<summary><strong>Permission Issues</strong></summary>

**Problem**: Can't control the turtle

**Solutions**:
- Ensure you're not running multiple turtlesim instances
- Check if another process is controlling the turtle
- Restart turtlesim if commands aren't working

</details>

### 💡 Still Stuck?

If you're still having issues, check the official documentation for your ROS version:

- **[ROS1 Noetic Installation](https://wiki.ros.org/noetic/Installation)** | **[Turtlesim Wiki](https://wiki.ros.org/turtlesim)**
- **[ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)** | **[Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)**
- **[ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation.html)** | **[Turtlesim Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)**

**Additional Resources:**
- **[ROS Answers](https://answers.ros.org/)** - Community Q&A for specific problems
- **ROS2 Doctor**: Run `ros2 doctor --report` to diagnose ROS2 installation issues
- **[ROS Troubleshooting Guide](https://wiki.ros.org/ROS/Troubleshooting)** (ROS1)