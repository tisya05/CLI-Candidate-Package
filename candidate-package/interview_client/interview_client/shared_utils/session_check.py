import os
from interview_client.shared_utils.session import load_session
from interview_client.shared_utils.client_config import LOCAL_FOLDER

def session_required_command(func):
    def wrapper(args):
        # Check for flag file
        flag_dir = os.path.expanduser("~/.interview_flags")
        active_flag = os.path.join(flag_dir, ".interview_active")

        if not os.path.exists(active_flag):
            print("\n[WARNING] No active interview session detected.")
            print("          Use `interview start <candidate_id>` to begin.\n")
            return

        if not LOCAL_FOLDER or not os.path.exists(LOCAL_FOLDER):
            print("\n[WARNING] Interview folder missing or corrupted.")
            print("          Run command inside a valid interview folder.\n")
            return

        session = load_session()
        if not session.get("candidate_id") or not session.get("task_id"):
            print("\n[WARNING] Incomplete session data.")
            print("          Please restart the interview session.\n")
            return

        return func(args)

    return wrapper
