import json
import os
from datetime import datetime
from interview_client.shared_utils.interview_path import get_interview_root
from interview_client.shared_utils.client_config import FAILURE_LOG_PATH


from interview_client.shared_utils.client_config import INTERVIEW_FLAG_DIR

def get_session_path():
    return os.path.join(get_interview_root(), "session.json")


def save_session(candidate_id, task_id, **kwargs):
    session_data = {
        "candidate_id": candidate_id,
        "task_id": task_id,
        **kwargs
    }
    path = get_session_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        with open(path, "w") as f:
            json.dump(session_data, f, indent=2)
    except Exception as e:
        log_failure("save_session", e)

def load_session():
    path = get_session_path()
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                return json.load(f)
        except Exception as e:
            log_failure("load_session", e)
    return {}

def clear_session():
    path = get_session_path()
    if os.path.exists(path):
        try:
            os.remove(path)
        except Exception as e:
            log_failure("clear_session", e)

def append_submission(files):
    session = load_session()
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    if "submissions" not in session or not isinstance(session["submissions"], list):
        session["submissions"] = []
    session["submissions"].append({
        "time": now,
        "files": files
    })
    save_session(
        candidate_id=session.get("candidate_id"),
        task_id=session.get("task_id"),
        **{k: v for k, v in session.items() if k not in ("candidate_id", "task_id")}
    )

def finalize_session(autosubmitted="no"):
    session = load_session()
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    session["autosubmitted"] = autosubmitted
    session["final_submission_time"] = now

    start_time = session.get("start_time")
    if start_time:
        try:
            start_dt = datetime.strptime(start_time, "%Y-%m-%d %H:%M:%S")
            end_dt = datetime.strptime(now, "%Y-%m-%d %H:%M:%S")
            duration = end_dt - start_dt
            session["total_duration_hms"] = (
                f"{duration.seconds // 3600} hr {(duration.seconds // 60) % 60} min {duration.seconds % 60} sec"
            )
        except Exception:
            session["total_duration_hms"] = "Invalid start_time format"

    try:
        path = get_session_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(session, f, indent=2)
    except Exception as e:
        log_failure("finalize_session", e)

def session_exists():
    try:
        return os.path.exists(get_session_path())
    except RuntimeError:
        return False

def log_failure(context, exception):
    try:
        with open(FAILURE_LOG_PATH, "a") as log:
            log.write(f"[{datetime.now()}] Error in {context}:\n{str(exception)}\n\n")
    except:
        pass  # Silent fail to avoid cascading errors
