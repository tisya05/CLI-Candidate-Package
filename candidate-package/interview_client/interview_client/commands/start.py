import os
import time
import requests
import re
from pathlib import Path
from dotenv import load_dotenv

from interview_client.shared_utils import session
from interview_client.shared_utils.client_config import (
    SERVER_URL,
    INTERVIEW_FLAG_DIR
)

load_dotenv()

if not os.path.exists(INTERVIEW_FLAG_DIR):
    os.makedirs(INTERVIEW_FLAG_DIR)

def check_for_reset_flag(candidate_id):
    flag_path = os.path.join(INTERVIEW_FLAG_DIR, f"{candidate_id}_reset.flag")
    if os.path.exists(flag_path):
        print(f"[INFO] Reset flag detected for {candidate_id}. Performing full reset.")
        from interview_client.shared_utils.full_reset import perform_full_reset
        try:
            perform_full_reset()
            os.remove(flag_path)
            print("[INFO] Reset complete. You can now start a fresh interview.")
        except Exception as e:
            print(f"[ERROR] Failed during reset: {e}")
        return True
    return False

def is_home_directory():
    return Path.cwd().resolve() == Path.home().resolve()

def is_effectively_empty_dir(path):
    ignored = {".DS_Store", ".gitkeep"}
    try:
        return all(f in ignored for f in os.listdir(path))
    except Exception:
        return False

def is_valid_email(email):
    return re.match(r"[^@]+@[^@]+\.[^@]+", email) is not None

def fetch_server_assignment(roll_number, email):
    try:
        url = f"{SERVER_URL}/assignment/{roll_number}"
        response = requests.get(url)

        if response.status_code == 200:
            return response.json().get("assignment")

        elif response.status_code == 404:
            print(f"[INFO] No assignment found, attempting to assign one for candidate: {roll_number}")
            start_url = f"{SERVER_URL}/start/{roll_number}"
            resp = requests.get(start_url, params={"email": email, "roll_number": roll_number})

            if resp.status_code == 200:
                response = requests.get(url)
                if response.status_code == 200:
                    return response.json().get("assignment")
                else:
                    print(f"[ERROR] Failed to fetch assignment after auto-assign. ({response.status_code})")
                    return None
            else:
                print(f"[ERROR] Failed to auto-assign task. Server said: {resp.text}")
                return None

        else:
            print(f"[ERROR] Unexpected response from assignment endpoint: {response.status_code}")
            return None

    except Exception as e:
        print(f"[ERROR] Failed to fetch or assign task: {e}")
        return None


def download_task_from_server(roll_number, task_id):
    try:
        response = requests.get(
            f"{SERVER_URL}/download-task",
            params={"candidate_id": roll_number, "task_id": task_id}
        )
        if response.status_code == 200:
            zip_path = os.path.join(Path.cwd(), "task.zip")
            with open(zip_path, "wb") as f:
                f.write(response.content)

            import zipfile
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall("task")
            os.remove(zip_path)
            return True
        else:
            print("[ERROR] Server failed to deliver task.")
            return False
    except Exception as e:
        print(f"[ERROR] Task download failed: {e}")
        return False

# ... (imports and existing code remain unchanged above)

def run(args):
    cwd = Path.cwd().resolve()

    if is_home_directory():
        print("[ERROR] Cannot start an interview in your home directory.")
        return False

    roll_number = args.candidate_id
    email = args.email

    if not roll_number:
        print("[ERROR] Roll number cannot be empty.")
        return False

    if not is_valid_email(email):
        print("[ERROR] Invalid email address format.")
        return False

    if check_for_reset_flag(roll_number):
        return False

    if not is_effectively_empty_dir(cwd):
        print("[ERROR] Cannot start interview in a non-empty directory.")
        return False

    print(f"\n--- Starting interview for roll number: {roll_number} ---\n")

    # Fetch assignment from server with email verification
    assignment = fetch_server_assignment(roll_number, email)
    if not assignment:
        print("[ERROR] No valid assignment found. Please contact staff.")
        return False

    # ✅ New: Email match check against assignment record
    registered_email = assignment.get("email")
    if registered_email != email:
        print("[ERROR] The provided email does not match our records.")
        return False

    chosen_task = assignment["task_id"]
    chosen_difficulty = assignment["difficulty"]
    print(f" => Selected task: {chosen_task} (difficulty: {chosen_difficulty})\n")

    LOCAL_FOLDER = os.path.join(cwd, "task")
    os.makedirs(LOCAL_FOLDER, exist_ok=True)

    success = download_task_from_server(roll_number, chosen_task)
    if not success:
        print("[ERROR] Failed to download task from server.")
        return False

    open(os.path.join(cwd, ".interview"), "w").close()

    session_data = {
        "candidate_id": roll_number,
        "email": email,
        "task_id": chosen_task,
        "start_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "submissions": [],
        "autosubmitted": "no"
    }
    session.save_session(**session_data)

    print(f"\n[INFO] Task has been downloaded to `{LOCAL_FOLDER}`.\n")
    print(" If you create new files and folders, make sure they are in that folder.\n")
    print(" The folder contains a README explaining the task.\n")
    print(" Available commands:\n")
    print("  – interview test          Run local tests on your code")
    print("  – interview test-all      Run hidden tests and get a report")
    print("  – interview submit        Submit your code and end the interview")
    print("  – interview --help        View full CLI command list\n")
    return True
