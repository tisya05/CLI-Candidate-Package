import argparse
import os
import subprocess
import sys

from interview_client.commands import start, test, test_hidden, client_submit
from interview_client.shared_utils.client_config import TIMER_DURATION_MINUTES, ALERT_MINUTES


def main():
    parser = argparse.ArgumentParser(description="---- Interview CLI Tool ----")
    subparsers = parser.add_subparsers(dest="command", required=True)

    start_parser = subparsers.add_parser("start", help="Start an interview session")
    # Remove the candidate_id argument — prompt interactively now

    def start_with_timer(args):
        # Prompt the user for Roll Number and Email
        candidate_id = input("Enter your Roll Number: ").strip()
        email = input("Enter your Email: ").strip()
        
        # Attach to args so it can be used in the start module
        args.candidate_id = candidate_id
        args.email = email

        started = start.run(args)
        if not started:
            return

        # Clean up any existing flags
        stop_file = os.path.join(os.getcwd(), "stop_timer.flag")
        if os.path.exists(stop_file):
            os.remove(stop_file)

        submitted_flag = os.path.join(os.getcwd(), ".submitted")
        if os.path.exists(submitted_flag):
            os.remove(submitted_flag)

        # Start timer
        duration_minutes = TIMER_DURATION_MINUTES
        alert_minutes = ALERT_MINUTES
        duration_sec = 60 * duration_minutes
        alert_intervals = [str(m * 60) for m in alert_minutes]

        try:
            subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "interview_client.shared_utils.timer",
                    str(duration_sec),
                    *alert_intervals,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            print("=========Timer has begun.=========")
        except Exception as e:
            print(f"Failed to launch timer: {e}")

    start_parser.set_defaults(func=start_with_timer)

    # Submit
    def submit_and_report(args):
        client_submit.run(args)

    submit_parser = subparsers.add_parser("submit", help="Submit one or more files")
    submit_parser.add_argument("--auto", action="store_true", help="Auto-confirm submission")
    submit_parser.set_defaults(func=submit_and_report)

    # Test
    test_parser = subparsers.add_parser("test", help="Run local tests (e.g. test.py)")
    test_parser.set_defaults(func=test.run)

    # Hidden Test
    hidden_test_parser = subparsers.add_parser("test-all", help="Run all tests and generate report")
    hidden_test_parser.set_defaults(func=test_hidden.run)

    # Parse and execute
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
