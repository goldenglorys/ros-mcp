#!/usr/bin/env python3
import json
import subprocess
import time
import os
import sys
from datetime import datetime

# Load tasks
with open('tasks.json', 'r') as f:
    tasks = json.load(f)

# Config
NUM_TRIALS = 3  # Runs per task
LLM_CLIENT = "claude"  # "claude" or "gemini"
LOGS_DIR = "experiment_logs"
os.makedirs(LOGS_DIR, exist_ok=True)

def launch_ros_setup():
    """Launch TurtleSim + rosbridge + MCP server in background."""
    # Terminal 1: Source ROS2 + TurtleSim
    turtlesim_proc = subprocess.Popen([
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node'
    ])
    time.sleep(2)
    
    # Terminal 2: rosbridge
    rosbridge_proc = subprocess.Popen([
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
    ])
    time.sleep(2)
    
    # Terminal 3: MCP server (uv run from parent dir)
    server_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    mcp_proc = subprocess.Popen([
        'bash', '-c', f'cd {server_dir} && uv run server.py'
    ])
    time.sleep(2)
    return [turtlesim_proc, rosbridge_proc, mcp_proc]

def run_task_with_claude(task_id, prompt, trial_num):
    """Interactive run with Claude Desktop: Prompt user to input in Claude, capture output."""
    print(f"\n--- Task {task_id} Trial {trial_num} ---")
    print(f"Prompt: {prompt}")
    input("Open Claude Desktop, paste prompt, run until done. Press Enter when trajectory complete (check logs).")
    
    # Manual log capture: Assume user pastes Claude's final response/trajectory here
    trajectory_input = input("Paste Claude's final response/trajectory JSON (or describe): ")
    try:
        trajectory = json.loads(trajectory_input) if trajectory_input.strip().startswith('{') else {"description": trajectory_input}
    except:
        trajectory = {"description": trajectory_input, "tools_used": []}  # Fallback
    
    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "task_id": task_id,
        "prompt": prompt,
        "trial": trial_num,
        "trajectory": trajectory,
        "success": input("Success? (y/n): ").lower() == 'y',  # Manual flag
        "notes": input("Notes (e.g., steps, grounding): ")
    }
    return log_entry

def run_task_with_gemini(task_id, prompt, trial_num):
    """Fallback: Use Gemini CLI for automated prompt (requires google-generativeai)."""
    import google.generativeai as genai
    genai.configure(api_key="YOUR_GEMINI_API_KEY")  # Set env var
    model = genai.GenerativeModel('gemini-1.5-flash')
    
    print(f"\n--- Task {task_id} Trial {trial_num} ---")
    print(f"Prompt: {prompt} (with MCP context: connect to localhost:9090)")
    
    response = model.generate_content(prompt + "\nUse ROS MCP tools to execute.")
    trajectory = {"response": response.text, "tools_used": []}  # Parse tools if needed
    
    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "task_id": task_id,
        "prompt": prompt,
        "trial": trial_num,
        "trajectory": trajectory,
        "success": "success" in response.text.lower(),  # Simple heuristic
        "notes": "Gemini auto-run"
    }
    return log_entry

def save_log(log_entry, task_id):
    log_file = os.path.join(LOGS_DIR, f"task_{task_id}_logs.jsonl")
    with open(log_file, 'a') as f:
        f.write(json.dumps(log_entry) + '\n')

def main():
    procs = launch_ros_setup()
    try:
        for task in tasks:
            for trial in range(1, NUM_TRIALS + 1):
                if LLM_CLIENT == "claude":
                    log_entry = run_task_with_claude(task["id"], task["prompt"], trial)
                else:
                    log_entry = run_task_with_gemini(task["id"], task["prompt"], trial)
                save_log(log_entry, task["id"])
                time.sleep(5)  # Cooldown between trials
    finally:
        for proc in procs:
            proc.terminate()

if __name__ == "__main__":
    main()