# ROS-MCP

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![GitHub Repo stars](https://img.shields.io/github/stars/goldenglorys/ros-mcp?style=social)
![GitHub last commit](https://img.shields.io/github/last-commit/goldenglorys/ros-mcp)

<p align="center">
  <img src="https://github.com/goldenglorys/ros-mcp/blob/main/docs/images/framework.png"/>
</p>

ROS-MCP connects language models (e.g Claude, GPT, and Gemini) with existing robots giving them bidirectional AI integration using natural language to control in ROS/ROS2.

### Key Benefits

- **No code changes** → Uses `rosbridge` for integration.
- **Bidirectional control** → Command and monitor bot state as it happening in ROS (sensors, topics, parameters).
- **ROS1 & ROS2 support** → Works with both versions.
- **MCP-compatible** → Supports any MCP-enabled LLM (Claude Desktop, Gemini, ChatGPT, and beyond).

Control Turtlesim with natural language commands like "Move the turtle forward."

## Getting Started

The server works with ROS1 or ROS2 and any MCP-enabled client.

### Installation

Follow the [installation guide](docs/installation.md):

1. Clone the repository
2. Install `uv` and `rosbridge`
3. Configure Gemini CLI or Claude Desktop (or any MCP-enabled client)
4. Launch `rosbridge` and Turtlesim

## Examples

- Try commands like "Draw a square with the turtle."
- See more in the [examples](turtlesim/tasks.json) folder.
