import base64
import json
import os
import requests

def delete_folder_from_github(owner, repo, folder_path, token, branch="main"):
    """Deletes all files in a GitHub folder (non-recursive)."""
    headers = {
        "Authorization": f"token {token}",
        "Accept": "application/vnd.github.v3+json"
    }

    api_url = f"https://api.github.com/repos/{owner}/{repo}/contents/{folder_path}"
    resp = requests.get(api_url, headers=headers)

    if resp.status_code != 200:
        print(f"[WARNING] Could not list files in '{folder_path}' for deletion. Status code: {resp.status_code}")
        return

    for item in resp.json():
        if item['type'] == 'file':
            delete_url = f"https://api.github.com/repos/{owner}/{repo}/contents/{item['path']}"
            response = requests.delete(delete_url, headers=headers, json={
                "message": f"Deleted file {item['name']} for overwrite",
                "sha": item['sha'],
                "branch": branch
            })
            if response.status_code not in [200, 204]:
                print(f"[ERROR] Failed to delete '{item['path']}': {response.status_code} - {response.text}")

def get_folder_list_from_github(owner, repo, path, token=None):
    """Returns a list of directory names at the given path in the repo."""
    headers = {"Authorization": f"token {token}"} if token else {}

    url = f"https://api.github.com/repos/{owner}/{repo}/contents/{path}"
    response = requests.get(url, headers=headers)

    if response.status_code != 200:
        print(f"[ERROR] Failed to fetch folder list at '{path}'. Status code: {response.status_code}")
        return []

    return [item['name'] for item in response.json() if item['type'] == 'dir']

def download_folder_from_github(owner, repo, remote_path, local_path, token=None):
    """Recursively downloads a GitHub folder's contents into a local directory."""
    headers = {"Authorization": f"token {token}"} if token else {}

    def _download(current_remote_path, current_local_path):
        os.makedirs(current_local_path, exist_ok=True)
        url = f"https://api.github.com/repos/{owner}/{repo}/contents/{current_remote_path}"
        response = requests.get(url, headers=headers)

        if response.status_code != 200:
            print(f"[ERROR] Failed to fetch contents of '{current_remote_path}'.")
            return False

        success = False
        for item in response.json():
            if item["type"] == "file":
                file_data = requests.get(item["download_url"], headers=headers)
                if file_data.status_code != 200:
                    print(f"[ERROR] Failed to download '{item['download_url']}' (Status: {file_data.status_code})")
                    continue
                with open(os.path.join(current_local_path, item["name"]), "wb") as f:
                    f.write(file_data.content)
                print(f"[INFO] Downloaded: {item['name']}")
                success = True
            elif item["type"] == "dir":
                if _download(item["path"], os.path.join(current_local_path, item["name"])):
                    success = True
        return success

    return _download(remote_path, local_path)

def get_file_sha(owner, repo, repo_path, headers, branch="main"):
    """Returns SHA of a file in GitHub repo (used for updates)."""
    url = f"https://api.github.com/repos/{owner}/{repo}/contents/{repo_path}?ref={branch}"
    resp = requests.get(url, headers=headers)
    return resp.json().get("sha") if resp.status_code == 200 else None

def push_file_to_github_api(owner, repo, local_file_path, repo_file_path, commit_message, token, branch="main"):
    """Pushes a single file to GitHub repo, replacing it if it already exists."""
    headers = {
        "Authorization": f"token {token}",
        "Accept": "application/vnd.github+json"
    }

    with open(local_file_path, "rb") as f:
        content_b64 = base64.b64encode(f.read()).decode()

    sha = get_file_sha(owner, repo, repo_file_path, headers, branch)

    data = {
        "message": commit_message,
        "content": content_b64,
        "branch": branch,
    }
    if sha:
        data["sha"] = sha

    url = f"https://api.github.com/repos/{owner}/{repo}/contents/{repo_file_path}"
    resp = requests.put(url, headers=headers, json=data)

    if resp.status_code not in [200, 201]:
        print(f"[ERROR] Failed to upload '{repo_file_path}': {resp.status_code} - {resp.text}")

def push_folder_to_github_api(owner, repo, local_folder_path, repo_folder_path, commit_message, token, files_to_push=None, branch="main"):
    """Pushes all files in a local folder to a GitHub folder path."""
    headers = {
        "Authorization": f"token {token}",
        "Accept": "application/vnd.github.v3+json"
    }

    if files_to_push is None:
        files_to_push = [
            (os.path.join(local_folder_path, f), f)
            for f in os.listdir(local_folder_path)
            if os.path.isfile(os.path.join(local_folder_path, f))
        ]

    success = True
    for local_file_path, repo_file_name in files_to_push:
        repo_file_path = f"{repo_folder_path}/{repo_file_name}"

        with open(local_file_path, "rb") as file:
            content_b64 = base64.b64encode(file.read()).decode("utf-8")

        url = f"https://api.github.com/repos/{owner}/{repo}/contents/{repo_file_path}"
        sha = get_file_sha(owner, repo, repo_file_path, headers, branch)

        data = {
            "message": commit_message,
            "content": content_b64,
            "branch": branch
        }
        if sha:
            data["sha"] = sha

        response = requests.put(url, headers=headers, data=json.dumps(data))
        if response.status_code not in [200, 201]:
            print(f"[ERROR] Failed to upload '{repo_file_path}': {response.status_code} - {response.json().get('message')}")
            success = False

    return success
