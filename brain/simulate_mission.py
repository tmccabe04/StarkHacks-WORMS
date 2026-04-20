import os
import sys

def send_mission(pid, mission):
    pipe_path = f"/tmp/worm_pipe_{pid}"
    if os.path.exists(pipe_path):
        with open(pipe_path, 'w') as f:
            f.write(mission + '\n')
        print(f"Sent mission '{mission}' to minion {pid}")
    else:
        print(f"Error: No pipe found for minion {pid}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 simulate_mission.py <pid> <mission>")
    else:
        send_mission(sys.argv[1], sys.argv[2])
