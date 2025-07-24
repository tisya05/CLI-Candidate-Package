import os
import json
import requests
from datetime import datetime, timezone
from interview_client.shared_utils.client_config import (
    IGNORED_FILES, IGNORED_EXTENSIONS, SERVER_URL
)
from interview_client.shared_utils.session import load_session, save_session, get_session_path
from interview_client.shared_utils.session_check import session_required_command

def get_local_folder():
    from interview_client.shared_utils.client_config import LOCAL_FOLDER
    if not LOCAL_FOLDER:
        raise RuntimeError("[ERROR] Not inside an active interview session.")
    return LOCAL_FOLDER

def get_next_report_path(reports_dir, report_type):
    os.makedirs(reports_dir, exist_ok=True)
    existing = [
        f for f in os.listdir(reports_dir)
        if f.startswith(f"{report_type}_report_") and f.endswith(".json")
    ]
    next_index = len(existing) + 1
    return os.path.join(reports_dir, f"{report_type}_report_{next_index}.json")

#@session_required_command
def run(args):
    local_folder = get_local_folder()

    if os.path.exists(".submitted") or os.path.exists(".stop_tests"):
        return

    session_data = load_session()
    candidate_id = session_data.get("candidate_id")
    task_id = session_data.get("task_id")

    print("[INFO] Collecting files for submission...")

    all_files = [
        f for f in os.listdir(local_folder)
        if os.path.isfile(os.path.join(local_folder, f))
    ]

    filtered_files = [
        f for f in all_files
        if f not in IGNORED_FILES and not any(f.endswith(ext) for ext in IGNORED_EXTENSIONS)
    ]

    session_path = get_session_path()
    if not os.path.exists(session_path):
        print("[ERROR] session.json not found.")
        return


    print("[INFO] Sending code to server for evaluation...")

    files = {f: open(os.path.join(local_folder, f), 'rb') for f in filtered_files}
    files["session.json"] = open(session_path, 'rb')

    try:
        if os.path.exists(".stop_tests"):
            return
        response = requests.post(f"{SERVER_URL}/test_hidden", files=files)
    finally:
        for f in files.values():
            f.close()

    if response.status_code != 200:
        print("[ERROR] Server returned:", response.text)
        return

    try:
        report_data = response.json()
    except Exception as e:
        print("[ERROR] Failed to parse server response:", e)
        return

    results = report_data.get("results", [])
    passed = sum(1 for r in results if r.get("passed") is True)
    total = len(results)

    print("\n--- Hidden Test Results ---")
    print("=" * 60)
    for r in results:
        test_name = r["test"].split("::")[-1]
        print(f"[{test_name}] => {'PASSED' if r['passed'] else 'FAILED'}")
    print("=" * 60)

    print("\n--- Summary ---")
    print(f"Passed: {passed}/{total}")
    print(">>> Status: All tests passed" if passed == total else ">>> Status: Some tests failed")

    reports_dir = os.path.join(os.path.dirname(local_folder), "reports")
    report_path = get_next_report_path(reports_dir, "hidden")

    with open(report_path, "w") as f:
        json.dump(report_data, f, indent=2)

    print(f"\n[INFO] Hidden test report saved at:\n>> {report_path}")

    session_data.setdefault("submissions", []).append({
        "time": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
        "note": "Hidden test run"
    })

    save_session(
        candidate_id=candidate_id,
        task_id=task_id,
        **{k: v for k, v in session_data.items() if k not in ("candidate_id", "task_id")}
    )

    print("[SUCCESS] Hidden test run complete.\n")
