#!/usr/bin/env python3
import json
import pandas as pd
import os
import subprocess  # For simple LLM judge if needed

LOGS_DIR = "experiment_logs"
TASKS_FILE = "tasks.json"

# Load tasks for expected_tools
with open(TASKS_FILE, 'r') as f:
    tasks = {t["id"]: t for t in json.load(f)}

def load_logs():
    """Load all .jsonl logs into DataFrame."""
    all_logs = []
    for log_file in os.listdir(LOGS_DIR):
        if log_file.endswith('.jsonl'):
            with open(os.path.join(LOGS_DIR, log_file), 'r') as f:
                for line in f:
                    all_logs.append(json.loads(line.strip()))
    return pd.DataFrame(all_logs)

def compute_metrics(df):
    """Compute: success rate, avg steps (heuristic: len(tools_used) or parse notes), grounding (0-10 heuristic)."""
    metrics = []
    for task_id, group in df.groupby('task_id'):
        task = tasks[task_id]
        success_rate = group['success'].mean() * 100
        avg_steps = group['trajectory'].apply(lambda t: len(t.get('tools_used', [])) if isinstance(t, dict) else 0).mean()
        # Simple grounding: 10 if 'pose' in notes/response, else 5
        grounding_scores = []
        for _, row in group.iterrows():
            notes = row['notes'].lower()
            traj = row['trajectory']
            score = 10 if 'pose' in notes or ('x' in str(traj) and 'y' in str(traj)) else 5
            grounding_scores.append(score)
        avg_grounding = sum(grounding_scores) / len(grounding_scores)
        
        metrics.append({
            "task_id": task_id,
            "prompt": task["prompt"][:50] + "...",
            "complexity": task["complexity"],
            "success_rate_%": round(success_rate, 1),
            "avg_steps": round(avg_steps, 1),
            "avg_grounding_0-10": round(avg_grounding, 1),
            "trials": len(group)
        })
    
    return pd.DataFrame(metrics)

def main():
    df = load_logs()
    metrics_df = compute_metrics(df)
    
    # Output table
    print(metrics_df.to_string(index=False))
    
    # Save CSV for thesis
    metrics_df.to_csv('evaluation_results.csv', index=False)
    
    # Overall summary
    overall_success = df['success'].mean() * 100
    print(f"\nOverall Success Rate: {overall_success:.1f}% across {len(df)} trials.")

if __name__ == "__main__":
    main()