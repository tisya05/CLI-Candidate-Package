import os
from pathlib import Path

def get_interview_root():
    current = os.getcwd()
    while current != os.path.dirname(current):  # until we reach the filesystem root
        if os.path.exists(os.path.join(current, ".interview")):
            return current
        current = os.path.dirname(current)
    return None

def get_local_task_folder():
    root = get_interview_root()
    if not root:
        raise RuntimeError("[ERROR] Invalid directory. Please run this command from a valid interview workspace.")
    return os.path.join(root, "task")
