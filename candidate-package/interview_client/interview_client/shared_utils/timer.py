import os
import sys
import customtkinter as ctk
import threading
import time
import platform
import subprocess
import json

from interview_client.shared_utils.client_config import LOCAL_FOLDER, INTERVIEW_FLAG_DIR
from interview_client.shared_utils.session import get_session_path

from interview_client.shared_utils.full_reset import perform_full_reset
from interview_client.shared_utils.client_config import GLOBAL_SESSION_LOCK

try:
    from pync import Notifier
except ImportError:
    Notifier = None


class TimerApp(ctk.CTk):
    def __init__(self, duration_sec, alert_intervals=None):
        super().__init__()
        self.duration = duration_sec
        self.alert_intervals = list(map(int, alert_intervals)) if alert_intervals else []
        self.title("Interview Timer")
        self.geometry("300x150")
        self.attributes("-topmost", True)
        self.protocol("WM_DELETE_WINDOW", self.on_close_attempt)

        self.label = ctk.CTkLabel(self, text=f"Time remaining: {self.format_time(self.duration)}", font=("Arial", 20))
        self.label.pack(pady=40)

        self.alerted = set()
        self._stop_event = threading.Event()
        self._flag_poll_notified = False  # prevent repeat notifications

        threading.Thread(target=self.countdown, daemon=True).start()
        threading.Thread(target=self.watch_for_submit, daemon=True).start()
        threading.Thread(target=self.listen_for_add_time_flag, daemon=True).start()
        threading.Thread(target=self.listen_for_reset_flag, daemon=True).start()

    def on_close_attempt(self):
        self.send_popup("You cannot close this window during the interview.")

    def watch_for_submit(self):
        submitted_path = os.path.join(LOCAL_FOLDER, ".submitted")
        while not self._stop_event.is_set():
            if os.path.exists(submitted_path):
                self._stop_event.set()
                self.send_popup("Submitting your code...")
                self.after(0, self.destroy)
                return
            time.sleep(0.2)

    def listen_for_add_time_flag(self):
        while not self._stop_event.is_set():
            try:
                candidate_id = self.get_candidate_id_from_session()
                if not candidate_id:
                    time.sleep(1)
                    continue

                flag_path = os.path.join(INTERVIEW_FLAG_DIR, f"{candidate_id}_add_time.flag")

                if not self._flag_poll_notified:
                    self._flag_poll_notified = True

                if os.path.exists(flag_path):
                    with open(flag_path) as f:
                        content = f.read().strip()

                    try:
                        minutes = int(content)
                        self.duration += minutes * 60
                        self.send_popup(f"Time extended by {minutes} minute(s).")
                        os.remove(flag_path)
                    except ValueError:
                        self.send_popup("Invalid time value in flag file.")
                        os.remove(flag_path)

                time.sleep(1)

            except Exception as e:
                self.send_popup(f"Timer listener error: {str(e)}")
                time.sleep(2)

    def listen_for_reset_flag(self):
        while not self._stop_event.is_set():
            try:
                candidate_id = self.get_candidate_id_from_session()
                if not candidate_id:
                    time.sleep(1)
                    continue

                flag_path = os.path.join(INTERVIEW_FLAG_DIR, f"{candidate_id}_reset.flag")
                if os.path.exists(flag_path):
                    self._stop_event.set()
                    self.send_popup("Session reset remotely. Closing interview.")
                    perform_full_reset()

                    try:
                        os.remove(flag_path)
                    except Exception as e:
                        print(f"[WARNING] Could not delete reset flag: {e}")

                    if os.path.exists(GLOBAL_SESSION_LOCK):
                        os.remove(GLOBAL_SESSION_LOCK)

                    self.after(0, self.destroy)
                    return

                time.sleep(1)
            except Exception as e:
                print(f"[ERROR] in reset listener: {e}")
                time.sleep(2)

    def get_candidate_id_from_session(self):
        try:
            session_path = get_session_path()
            with open(session_path) as f:
                session = json.load(f)
            return session.get("candidate_id")
        except Exception:
            return None

    def format_time(self, secs):
        return f"{secs // 60:02}:{secs % 60:02}"

    def send_popup(self, message):
        try:
            if platform.system() == "Darwin" and Notifier:
                Notifier.notify(message, title="Interview Timer")
            elif platform.system() == "Windows":
                from plyer import notification
                notification.notify(title="Interview Timer", message=message, timeout=5)
            elif platform.system() == "Linux":
                subprocess.call(['notify-send', 'Interview Timer', message])
        except Exception:
            pass

    def countdown(self):
        while self.duration > 0 and not self._stop_event.is_set():
            time.sleep(1)
            self.duration -= 1

            if self.duration in self.alert_intervals and self.duration not in self.alerted:
                mins = self.duration // 60
                self.send_popup(f"{mins} minute(s) remaining!")
                self.alerted.add(self.duration)

            self.label.configure(text=f"Time remaining: {self.format_time(self.duration)}")

        if not self._stop_event.is_set() and not os.path.exists(os.path.join(LOCAL_FOLDER, ".submitted")):
            self._stop_event.set()
            self.withdraw()
            self.send_popup("Time's up. Submitting your code...")

            def submit_and_notify():
                try:
                    subprocess.run(["interview", "submit", "--auto"], env={**os.environ, "INTERVIEW_AUTOSUBMIT": "1"})
                except Exception:
                    pass
                finally:
                    self.after(0, self.destroy)

            threading.Thread(target=submit_and_notify, daemon=True).start()


if __name__ == "__main__":
    ctk.set_appearance_mode("system")
    ctk.set_default_color_theme("blue")

    duration_sec = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    alert_intervals = list(map(int, sys.argv[2:])) if len(sys.argv) > 2 else []

    app = TimerApp(duration_sec, alert_intervals)
    app.mainloop()
    os._exit(0)
