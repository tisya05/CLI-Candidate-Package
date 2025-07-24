import os
import time
import requests
import sys
import json
import platform
import subprocess
from datetime import datetime
from interview_client.shared_utils.session import (
    load_session,
    append_submission,
    finalize_session,
    get_session_path
)
from interview_client.shared_utils.cleanup import cleanup
from interview_client.shared_utils.session_check import session_required_command
from interview_client.shared_utils.client_config import (
    SERVER_URL, IGNORED_FILES, IGNORED_EXTENSIONS, LOCAL_FOLDER,
    INTERVIEW_FLAG_DIR, FAILURE_LOG_PATH
)

def notify_user(message):
    try:
        if platform.system() == "Darwin":
            from pync import Notifier
            Notifier.notify(message, title="Interview Submission")
        elif platform.system() == "Windows":
            from plyer import notification
            notification.notify(title="Interview Submission", message=message, timeout=5)
        elif platform.system() == "Linux":
            subprocess.call(['notify-send', 'Interview Submission', message])
    except Exception:
        pass

def get_reports_folder():
    return os.path.abspath(os.path.join(LOCAL_FOLDER, "..", "reports"))

def is_solution_file(filename):
    fname = filename.lower()
    if fname.startswith(("readme", "test", "local_test.", "requirement")):
        return False
    if any(fname.endswith(ext) for ext in IGNORED_EXTENSIONS):
        return False
    if fname in IGNORED_FILES or fname.startswith("."):
        return False
    return True

def cleanup_local():
    cleanup(LOCAL_FOLDER, silent=True, log_if_fail=True)
    cleanup(get_reports_folder(), silent=True, log_if_fail=True)

    # Remove interview flag directory entirely
    if os.path.exists(INTERVIEW_FLAG_DIR):
        try:
            import shutil
            shutil.rmtree(INTERVIEW_FLAG_DIR)
        except Exception as e:
            print(f"[WARN] Failed to delete INTERVIEW_FLAG_DIR: {e}")

    # Remove other legacy flags outside the flag dir
    session_root = os.path.abspath(os.path.join(LOCAL_FOLDER, ".."))
    interview_flag = os.path.join(session_root, ".interview")
    if os.path.exists(interview_flag):
        os.remove(interview_flag)

#@session_required_command
def run(args):
    is_auto = getattr(args, "auto", False)

    if not os.path.isdir(LOCAL_FOLDER):
        print("[ERROR] Local task folder doesn't exist.")
        return

    all_files = [
        f for f in os.listdir(LOCAL_FOLDER)
        if os.path.isfile(os.path.join(LOCAL_FOLDER, f))
    ]

    display_files = [f for f in all_files if is_solution_file(f)]

    # Final files includes display files + report.json (if any)
    final_files = display_files[:]
    report_path = os.path.join(LOCAL_FOLDER, "report.json")
    if os.path.exists(report_path):
        final_files.append("report.json")

    if not display_files:
        print("[INFO] No solution files found to submit.")
        notify_user("No solution files found to submit.")
        return

    submitted_path = os.path.join(LOCAL_FOLDER, ".submitted")
    if os.path.exists(submitted_path):
        return

    if not is_auto:
        print("Files to submit:")
        for f in display_files:
            print(" -", f)
        while True:
            confirm = input("Are you sure you want to submit these files? (y/n): ").strip().lower()
            if confirm == "y":
                with open(submitted_path, "w") as f:
                    f.write("submitted")
                break
            elif confirm == "n":
                print("Submission cancelled.")
                return
            else:
                print("Please enter 'y' or 'n'.")
    else:
        with open(submitted_path, "w") as f:
            f.write("submitted")

    try:
        finalize_session(autosubmitted="yes" if is_auto else "no")
    except Exception as e:
        print("[ERROR] Failed to finalize session:", e)
        return

    print("[INFO] Submitting your code to the server...")
    notify_user("Submitting your code...")

    files = {}
    for fname in final_files:
        full_path = os.path.join(LOCAL_FOLDER, fname)

        # ✅ Skip unwanted extensions
        if any(fname.endswith(ext) for ext in IGNORED_EXTENSIONS):
            print(f"[DEBUG] Skipping ignored extension: {fname}")
            continue

        # ✅ Skip __pycache__ or similar folders (if walking full paths)
        if "__pycache__" in full_path:
            print(f"[DEBUG] Skipping file in __pycache__: {fname}")
            continue

        files[fname] = open(full_path, "rb")

    # ✅ Add session.json from actual session path
    session_path = get_session_path()
    if os.path.exists(session_path):
        files["session.json"] = open(session_path, "rb")

    try:
        response = requests.post(
            f"{SERVER_URL}/submit",
            files={name: (name, f) for name, f in files.items()}
        )

        if response.status_code == 200:
            try:
                data = response.json()
                if data.get("success"):
                    with open(report_path, "w") as f:
                        json.dump(data["report"], f, indent=2)

                    append_submission(files=display_files)

                    if os.path.exists(submitted_path):
                        os.remove(submitted_path)

                    print("[SUCCESS] Submission completed.")
                    cleanup_local()
                    notify_user("Submission successful.")
                else:
                    print("[ERROR] Submission failed:", data.get("error", "Unknown error."))
                    notify_user("Submission failed.")
            except Exception:
                print("[ERROR] Invalid JSON response from server.")
                try:
                    with open(FAILURE_LOG_PATH, "w") as f:
                        f.write(f"[{datetime.now()}] Failed to parse server JSON.\n")
                        f.write(response.text + "\n")
                except Exception as log_err:
                    print("[ERROR] Could not write to failure log:", log_err)
                notify_user("Submission failed.")
        else:
            print(f"[ERROR] Server error {response.status_code}")
            print("[DEBUG] Response text:", response.text)
            notify_user("Submission failed.")
    except Exception as e:
        print("[ERROR] Submission failed:", e)
        notify_user("Submission failed.")
    finally:
        for f in files.values():
            f.close()
