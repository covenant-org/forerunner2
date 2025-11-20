## TODO: Instalar python, tkinter(python3-tk con apt), pip3 y rerun-sdk en docker
import os
from pathlib import Path
from typing import List, Optional, Union

import tkinter as tk
from tkinter import messagebox, ttk

try:
    from minio import Minio
    from minio.error import S3Error
except ImportError:  # pragma: no cover - optional dependency, handled at runtime.
    Minio = None  # type: ignore
    S3Error = Exception  # type: ignore


DEFAULT_MINIO_BUCKET = "rerun"
DEFAULT_MINIO_PREFIX = ""
DEFAULT_MINIO_ENDPOINT = "127.0.0.1:9000"
DEFAULT_MINIO_SECURE = False

def get_minio_client():
    """Devuelve una tupla (client, bucket, prefix) si Minio está configurado, o (None, None, None) si no."""
    if Minio is None or not (os.getenv("MINIO_ACCESS_KEY") and os.getenv("MINIO_SECRET_KEY")):
        return None, None, None
    endpoint = os.getenv("MINIO_ENDPOINT", DEFAULT_MINIO_ENDPOINT)
    access_key = os.getenv("MINIO_ACCESS_KEY")
    secret_key = os.getenv("MINIO_SECRET_KEY")
    secure = _parse_bool_env(os.getenv("MINIO_USE_HTTPS"), DEFAULT_MINIO_SECURE)
    bucket = os.getenv("MINIO_BUCKET", DEFAULT_MINIO_BUCKET)
    prefix = os.getenv("MINIO_PREFIX", DEFAULT_MINIO_PREFIX)
    client = Minio(endpoint=endpoint, access_key=access_key, secret_key=secret_key, secure=secure)
    return client, bucket, prefix

def list_test_records(records_dir: Union[str, Path, None] = None,
                      include_hidden: bool = False,
                      extensions: Optional[List[str]] = None) -> List[str]:
    if os.getenv("MINIO_ACCESS_KEY") and os.getenv("MINIO_SECRET_KEY"):
        return _list_records_from_minio(
            bucket=DEFAULT_MINIO_BUCKET,
            prefix=os.getenv("MINIO_PREFIX", DEFAULT_MINIO_PREFIX),
            include_hidden=include_hidden,
            extensions=extensions,
        )

    return _list_records_from_local(records_dir, include_hidden, extensions)


def _list_records_from_local(records_dir: Union[str, Path, None],
                             include_hidden: bool,
                             extensions: Optional[List[str]]) -> List[str]:
    if records_dir is None:
        base = Path(__file__).resolve().parent / "test-records"
    else:
        base = Path(records_dir)

    try:
        if not base.exists() or not base.is_dir():
            return []
    except Exception:
        return []

    normalized_exts = None
    if extensions:
        normalized_exts = [e.lower() if e.startswith('.') else f'.{e.lower()}' for e in extensions]

    names = []

    # Files in the root of test-records
    for p in base.iterdir():
        if p.is_file():
            name = p.name
            if not include_hidden and name.startswith('.'):
                continue
            if normalized_exts is not None:
                low = name.lower()
                if not any(low.endswith(ext) for ext in normalized_exts):
                    continue
            names.append(name)

        # If a directory is found, include its files (one level)
        elif p.is_dir():
            subdir = p
            for q in subdir.iterdir():
                if not q.is_file():
                    continue
                subname = q.name
                if not include_hidden and subname.startswith('.'):
                    continue
                if normalized_exts is not None:
                    low = subname.lower()
                    if not any(low.endswith(ext) for ext in normalized_exts):
                        continue
                # Format folder/filename.ext
                names.append(f"{subdir.name}/{subname}")

    names.sort()
    return names


def _parse_bool_env(value: Optional[str], default: bool) -> bool:
    if value is None:
        return default
    lowered = value.strip().lower()
    if lowered in {"1", "true", "yes", "on"}:
        return True
    if lowered in {"0", "false", "no", "off"}:
        return False
    return default


def _list_records_from_minio(bucket: str,
                             prefix: str,
                             include_hidden: bool,
                             extensions: Optional[List[str]]) -> List[str]:
    if Minio is None:
        print("minio SDK is not installed.")
        return []

    client, _, _ = get_minio_client()
    if client is None:
        print("Minio no está configurado correctamente.")
        return []

    normalized_exts = None
    if extensions:
        normalized_exts = [e.lower() if e.startswith('.') else f'.{e.lower()}' for e in extensions]

    results: List[str] = []
    prefix = prefix.lstrip('/')
    try:
        for obj in client.list_objects(bucket_name=bucket, prefix=prefix, recursive=True):
            name = obj.object_name
            if prefix and name.startswith(prefix):
                name = name[len(prefix):]
            name = name.lstrip('/')
            if not name or name.endswith('/'):
                continue
            if not include_hidden and any(part.startswith('.') for part in name.split('/')):
                continue
            if normalized_exts is not None:
                low = name.lower()
                if not any(low.endswith(ext) for ext in normalized_exts):
                    continue
            results.append(name)
    except S3Error as exc:
        print(f"Unable to list objects from bucket '{bucket}': {exc}")
        return []

    results.sort()
    return results

def _build_gui_for_records(records):
    def show_download_progress(client, bucket, object_name, local_path, fname):
        progress_win = tk.Toplevel()
        progress_win.title("Downloading...")
        progress_label = tk.Label(progress_win, text=f"Downloading: {fname}")
        progress_label.pack(padx=10, pady=10)
        progress_bar = ttk.Progressbar(progress_win, orient="horizontal", length=300, mode="determinate")
        progress_bar.pack(padx=10, pady=10)
        progress_bar['value'] = 0
        progress_win.update()

        try:
            stat = client.stat_object(bucket, object_name)
            total_size = stat.size
        except Exception as e:
            progress_win.destroy()
            messagebox.showerror("Download error", f"Could not get file size: {e}")
            return

        try:
            response = client.get_object(bucket, object_name)
            with open(local_path, "wb") as f:
                downloaded = 0
                chunk_size = 1024 * 1024  # 1MB
                while True:
                    data = response.read(chunk_size)
                    if not data:
                        break
                    f.write(data)
                    downloaded += len(data)
                    percent = (downloaded / total_size) * 100 if total_size else 0
                    progress_bar['value'] = percent
                    progress_win.update()
            response.close()
            progress_bar['value'] = 100
            progress_win.update()
            progress_win.destroy()
            messagebox.showinfo("Download complete", f"File saved at: {local_path}")
        except Exception as e:
            progress_win.destroy()
            messagebox.showerror("Download error", f"Could not download: {e}")

    def on_click(fname):
        client, bucket, prefix = get_minio_client()
        if client is not None:
            object_name = f"{prefix}/{fname}" if prefix else fname
            object_name = object_name.lstrip("/")
            base_dir = Path("/tmp/rerun-recordings")
            local_path = base_dir / fname
            local_path.parent.mkdir(parents=True, exist_ok=True)
            show_download_progress(client, bucket, object_name, local_path, fname)
        else:
            messagebox.showinfo("File selected", fname)

    def _on_double_click(event):
        # Action: double-click on an item calls on_click with the full path
        item_id = tree.focus()
        if not item_id:
            return
        vals = tree.item(item_id).get('values')
        if vals:
            full = vals[0]
            on_click(full)
    
    
    root = tk.Tk()
    root.title("Test-records")
    root.geometry("480x400")

    frm = tk.Frame(root)
    frm.pack(fill=tk.BOTH, expand=True)
    tree_frame = tk.Frame(frm)

    # Use a frame directly for the tree so it fills the window area
    # (previous implementation used a canvas+frame which limited the
    # treeview width/height inside the canvas window).

    # Group by folder ('' = root)
    groups = {}
    for name in records:
        if "/" in name:
            folder, fname = name.split("/", 1)
            groups.setdefault(folder, []).append(fname)
        else:
            groups.setdefault("", []).append(name)

    # Build a tree view with ttk.Treeview: root files as items
    # at the root level and folders as nodes containing their files.
    # tree_frame is already a child of frm; just pack it so it expands
    tree_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

    tree = ttk.Treeview(tree_frame)
    # Make the primary column expand to fill the available width
    tree.heading('#0', text='Files', anchor='w')
    tree.column('#0', anchor='w', stretch=True)

    # Scrollbar for the treeview
    tree_scroll = tk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=tree.yview)
    tree.configure(yscrollcommand=tree_scroll.set)
    tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

    # Insert root files
    root_files = groups.pop("", [])
    for name in sorted(root_files):
        # displayed text = name, values[0] = full path (same as name)
        tree.insert('', 'end', text=name, values=(name,))

    # Insert folders with their children
    for folder in sorted(groups.keys()):
        node = tree.insert('', 'end', text=f"{folder}/", open=False)
        items = sorted(groups[folder])
        for fname in items:
            full = f"{folder}/{fname}"
            tree.insert(node, 'end', text=fname, values=(full,))



    tree.bind('<Double-1>', _on_double_click)

    root.mainloop()


if __name__ == "__main__":
    files = list_test_records()
    # Call the GUI (assumed always available)
    _build_gui_for_records(files)