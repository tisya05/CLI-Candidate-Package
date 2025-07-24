import os
from dotenv import load_dotenv
from pathlib import Path
from interview_client.shared_utils.interview_path import get_interview_root, get_local_task_folder

load_dotenv()

INTERVIEW_ROOT = get_interview_root()
LOCAL_FOLDER = get_local_task_folder() if INTERVIEW_ROOT else None

INTERVIEW_FLAG_DIR = os.path.expanduser("~/.interview_flags")
FAILURE_LOG_PATH = os.path.join(INTERVIEW_FLAG_DIR, "submission_failure.log")
GLOBAL_SESSION_LOCK = os.path.join(INTERVIEW_FLAG_DIR, ".interview_active")

# DB and Server config
DB_URI = os.getenv("DB_URI")
SERVER_URL = "https://31769a53454f.ngrok-free.app"

# Timer config
TIMER_DURATION_MINUTES = 10
ALERT_MINUTES = [1]

# GitHub config
REPO_OWNER = "tisya05"
REPO_NAME = "Nexthop-Test-Repo"
FOLDER_PATH = "sample_questions"
HIDDEN_TESTS_FOLDER = "hidden_tests"
GITHUB_TOKEN = os.getenv("GITHUB_TOKEN")
SUBMISSIONS = "SUBMISSIONS"

# Files and folders to ignore during upload or cleanup
IGNORED_FILES = [
    "README.txt",
    "local_tests.py",
    "session.json",
    "report.json",
    ".DS_Store"
]

IGNORED_FOLDERS = [
    "__pycache__",
    ".git"
]

IGNORED_EXTENSIONS = [
    ".pyc",
    ".log"
]
