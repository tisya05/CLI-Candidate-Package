import os
import shutil
from interview_client.shared_utils.session import load_session, clear_session
from interview_client.shared_utils.cleanup import cleanup
from interview_client.shared_utils.interview_path import get_interview_root, get_local_task_folder

def perform_full_reset():
    # 1. Remove timer/flag files
    for file in ["stop_timer.flag", ".submitted", ".test_hidden", ".autosubmitted"]:
        try:
            if os.path.exists(file):
                os.remove(file)
        except:
            pass

    # 2. Load session
    session = load_session()
    candidate_id = session.get("candidate_id")
    task_id = session.get("task_id")

    if candidate_id and task_id:
        try:
            local_folder = get_local_task_folder()
            if os.path.exists(local_folder):
                cleanup(local_folder, silent=True, log_if_fail=False)
        except:
            pass

    clear_session()

    # 3. Delete `.interview` marker if present in root
    interview_root = get_interview_root()
    if interview_root:
        interview_flag = os.path.join(interview_root, ".interview")
        try:
            if os.path.exists(interview_flag):
                os.remove(interview_flag)
        except:
            pass

    # 4. Clean flag dir itself
    if interview_root:
        try:
            shutil.rmtree(interview_root)
        except:
            pass

    print("Interview session reset completed.")
