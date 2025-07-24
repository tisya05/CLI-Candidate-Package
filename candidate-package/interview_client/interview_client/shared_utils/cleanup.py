import shutil
import os
from interview_client.shared_utils.client_config import FAILURE_LOG_PATH

def cleanup(folder_path, silent=True, log_if_fail=False):
    """
    Deletes the folder at the given path.

    Args:
        folder_path (str): Path to the folder to delete.
        silent (bool): If False, print cleanup status to console.
        log_if_fail (bool): If True, log failures to submission_failure.log.
    """
    if os.path.exists(folder_path):
        try:
            shutil.rmtree(folder_path)
            if not silent:
                #print(f"[INFO] Cleaned up folder: {folder_path}")
                pass
        except Exception as e:
            msg = f"[ERROR] Cleanup failed for '{folder_path}': {e}"
            if not silent:
                print(msg)
            if log_if_fail:
                try:
                    with open(FAILURE_LOG_PATH, "a") as f:
                        f.write(msg + "\n")
                except Exception:
                    pass  # Silent fallback on logging error
    else:
        msg = f"[WARNING] Cleanup skipped. Folder does not exist: {folder_path}"
        if not silent:
            print(msg)
        if log_if_fail:
            try:
                with open(FAILURE_LOG_PATH, "a") as f:
                    f.write(msg + "\n")
            except Exception:
                pass  # Silent fallback on logging error
