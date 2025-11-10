## TODO: Instalar python, tkinter(python3-tk con apt), pip3 y rerun-sdk en docker

from pathlib import Path
from typing import List, Optional, Union

import tkinter as tk
from tkinter import messagebox, ttk


def list_test_records(records_dir: Union[str, Path, None] = None,
                      include_hidden: bool = False,
                      extensions: Optional[List[str]] = None) -> List[str]:
    ## TODO: Esta funcion se ejecuta en el servidor y los regresa al cliente
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

def _build_gui_for_records(records):
    def on_click(fname):
        ## TODO: Ejecutar rerun en servidor con ese archivo
        ## TODO: Ejecutar rerun cliente conectado a ip de servidor
        messagebox.showinfo("Archivo seleccionado", fname)

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
    tree.heading('#0', text='Archivos', anchor='w')
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