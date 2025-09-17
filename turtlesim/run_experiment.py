#!/usr/bin/env python3
import json
import subprocess
import time
import os
from datetime import datetime
import random
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from tqdm import tqdm

console = Console()

with open("tasks.json", "r") as f:
    tasks = json.load(f)

NUM_TRIALS = 1
LOGS_DIR = "experiment_logs"
RUN_ID = datetime.now().strftime(
    "%Y-%m-%dT%H:%M"
)
RUN_LOGS_DIR = os.path.join(LOGS_DIR, f"run_{RUN_ID}")
os.makedirs(RUN_LOGS_DIR, exist_ok=True)
GEMINI_CMD = ["gemini"] 
MODEL = "gemini-1.5-pro"  # Stable fallback; change if 2.5-pro available
PROVIDER = "google"
MAX_RETRIES = 3
BASE_BACKOFF = 5.0
REPO_ROOT = "/home/ubuntu/Documents/ros-mcp"


def launch_ros_setup():
    """Launch TurtleSim + rosbridge in background using bash -c."""
    os.chdir(REPO_ROOT)
    subprocess.Popen(
        [
            "bash",
            "-c",
            "source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2)
    subprocess.Popen(
        [
            "bash",
            "-c",
            "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2)
    console.print(
        Panel(
            "ROS setup launched. Gemini CLI will auto-start ros-mcp server.",
            title="Setup",
            style="bold green",
        )
    )


def run_task_gemini_cli(task_id, prompt, trial_num, last_state_summary=""):
    """Automate Gemini CLI with positional prompt, retries/backoff."""
    full_prompt = (
        f"{last_state_summary} "
        f"Connect to ros-mcp at ws://0.0.0.0:9090 (assume ros-mcp, no extra details needed). "
        f"Execute this fuzzy task using MCP tools (e.g., subscribe /turtle1/pose, publish /turtle1/cmd_vel, call /spawn for multiples). "
        f"Handle all nuances autonomously: multi-turn planning, services like /teleport_absolute or /set_pen, multi-turtle if needed (e.g., pub to /turtle2/cmd_vel). "
        f"Ground in outputs (cite poses). Output JSON only: {{'tools_used': ['list', 'of', 'tools'], 'response': 'summary', 'final_pose': {{'x': num, 'y': num}}, 'multi_turtle_poses': {{}} if applicable}}."
        f"Task: {prompt}"
    )

    console.print(f"\n--- [bold cyan]Task {task_id} Trial {trial_num}[/bold cyan] ---")
    console.print(f"Prompt Preview: {full_prompt[:100]}...")

    for attempt in range(1, MAX_RETRIES + 1):
        try:
            proc = subprocess.run(
                GEMINI_CMD + [full_prompt],
                text=True,
                capture_output=True,
                timeout=60,
                env={**os.environ, "GEMINI_API_KEY": os.getenv("GEMINI_API_KEY", "")},
                cwd=REPO_ROOT,
            )

            if proc.returncode == 0:
                stdout = proc.stdout.strip()
                console.print(f"[green]Success![/green] Raw Output: {stdout[:200]}...")
                try:
                    trajectory = (
                        json.loads(stdout)
                        if stdout.startswith("{")
                        else {
                            "response": stdout,
                            "tools_used": [
                                line
                                for line in stdout.split("\n")
                                if any(
                                    tool in line
                                    for tool in ["publish", "subscribe", "call_service"]
                                )
                            ],
                            "final_pose": {},
                            "multi_turtle_poses": {},
                        }
                    )
                except json.JSONDecodeError:
                    trajectory = {
                        "raw_response": stdout,
                        "tools_used": [],
                        "final_pose": {},
                        "multi_turtle_poses": {},
                    }

                success = any(
                    word in str(trajectory).lower()
                    for word in ["success", "done", "executed"]
                ) or "pose" in str(trajectory)
                steps = len(trajectory.get("tools_used", []))
                notes = f"Success on attempt {attempt}; {steps} steps"
                retry_attempts = attempt
                error_details = ""
                break
            else:
                error_details = proc.stderr
                console.print(
                    f"[bold red]Attempt {attempt} failed:[/bold red] {error_details[:100]}"
                )
                if attempt < MAX_RETRIES:
                    delay = (BASE_BACKOFF * (2 ** (attempt - 1))) + random.uniform(0, 1)
                    console.print(f"[yellow]Backoff {delay:.1f}s...[/yellow]")
                    time.sleep(delay)
                else:
                    trajectory = {"error": error_details, "tools_used": []}
                    success = False
                    steps = 0
                    notes = f"Failed after {MAX_RETRIES} attempts"
                    retry_attempts = MAX_RETRIES

        except subprocess.TimeoutExpired:
            error_details = "Timeout"
            console.print(f"[bold red]Attempt {attempt} timeout[/bold red]")
            if attempt < MAX_RETRIES:
                delay = (BASE_BACKOFF * (2 ** (attempt - 1))) + random.uniform(0, 1)
                time.sleep(delay)
            else:
                trajectory = {"error": "Max retries timeout", "tools_used": []}
                success = False
                steps = 0
                notes = "Max retries on timeout"
                retry_attempts = MAX_RETRIES

    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "task_id": task_id,
        "prompt": prompt,
        "trial": trial_num,
        "trajectory": trajectory,
        "success": success,
        "steps": steps,
        "notes": notes,
        "retry_attempts": retry_attempts,
        "error_details": error_details if "error_details" in locals() else "",
        "model": MODEL,
        "provider": PROVIDER,
        "run_id": RUN_ID,
    }
    return log_entry


def save_log(log_entry, task_id):
    log_file = os.path.join(RUN_LOGS_DIR, f"task_{task_id}_logs.jsonl")
    with open(log_file, "a") as f:
        f.write(json.dumps(log_entry) + "\n")


def main():
    launch_ros_setup()
    last_state_summary = ""
    total_tasks = len(tasks)
    task_successes = []

    with tqdm(total=total_tasks, desc="Running Tasks", unit="task") as pbar:
        try:
            for i, task in enumerate(tasks, 1):
                progress = (i / total_tasks) * 100
                pbar.set_description(
                    f"Running Task {i}/{total_tasks} ({progress:.1f}%)"
                )

                log_entry = run_task_gemini_cli(
                    task["id"], task["prompt"], 1, last_state_summary
                )
                save_log(log_entry, task["id"])

                success_str = (
                    "[green]Success[/green]"
                    if log_entry["success"]
                    else "[red]Failed[/red]"
                )
                pbar.set_postfix(
                    {
                        "Status": success_str,
                        "Steps": log_entry["steps"],
                        "Retries": log_entry["retry_attempts"],
                    }
                )
                task_successes.append(log_entry["success"])

                # Chain state
                last_state_summary = f"Previous state summary: {log_entry['trajectory'].get('response', 'N/A')[:100]} (final pose: {log_entry['trajectory'].get('final_pose', {})}). Build coherent workflow."

                time.sleep(15)  # Cooldown per task
                pbar.update(1)

        finally:
            os.system("pkill -f turtlesim_node")
            os.system("pkill -f rosbridge_server")
            console.print(
                Panel("Experiment Complete", title="Summary", style="bold green")
            )
            overall_success = (
                sum(task_successes) / len(task_successes) * 100 if task_successes else 0
            )
            console.print(
                f"Overall Success Rate: [bold magenta]{overall_success:.1f}%[/bold magenta] across {total_tasks} tasks."
            )
            console.print(
                f"Logs saved in [bold blue]{RUN_LOGS_DIR}/[/bold blue]. Run evaluate.py for full analysis."
            )

            table = Table(title="Run Summary")
            table.add_column("Task ID", style="cyan")
            table.add_column(
                "Success", style="green" if overall_success > 50 else "red"
            )
            table.add_column("Steps", justify="right")
            table.add_column("Retries", justify="right")
            table.add_column("Model", style="magenta")
            for entry in task_successes:
                table.add_row(
                    str(i),
                    "Yes" if entry else "No",
                    str(log_entry["steps"]),
                    str(log_entry["retry_attempts"]),
                    MODEL,
                )
            console.print(table)


if __name__ == "__main__":
    main()
