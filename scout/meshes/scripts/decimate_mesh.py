#!/usr/bin/env python3
import os
import shutil
import numpy as np
import trimesh
from fast_simplification import simplify as fs_simplify

MESHES_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BACKUP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backup')


def list_stl_files():
    return sorted(f for f in os.listdir(MESHES_DIR) if f.lower().endswith('.stl'))


def select_file(files):
    print("STL files in current directory:")
    for i, f in enumerate(files, 1):
        size = os.path.getsize(os.path.join(MESHES_DIR, f)) / 1024
        print(f"  {i}. {f}  ({size:.1f} KB)")

    while True:
        try:
            choice = int(input("\nSelect a file (number): "))
            if 1 <= choice <= len(files):
                return files[choice - 1]
        except ValueError:
            pass
        print(f"Enter a number between 1 and {len(files)}.")


def get_ratio():
    while True:
        try:
            raw = input("Target face ratio (0.0 - 1.0, default 0.5): ").strip()
            if raw == '':
                return 0.5
            ratio = float(raw)
            if 0.0 < ratio < 1.0:
                return ratio
        except ValueError:
            pass
        print("Enter a value between 0.0 and 1.0.")


def main():
    files = list_stl_files()
    if not files:
        print("No STL files found in the current directory.")
        return

    selected = select_file(files)
    ratio = get_ratio()

    src_path = os.path.join(MESHES_DIR, selected)
    mesh = trimesh.load(src_path)
    original_faces = len(mesh.faces)

    os.makedirs(BACKUP_DIR, exist_ok=True)
    backup_path = os.path.join(BACKUP_DIR, selected)
    shutil.move(src_path, backup_path)
    print(f"\nOriginal backed up to: {backup_path}")

    points_out, faces_out = fs_simplify(
        mesh.vertices.astype(np.float32),
        mesh.faces.astype(np.int32),
        target_reduction=1.0 - ratio,
    )
    decimated = trimesh.Trimesh(vertices=points_out, faces=faces_out)
    decimated.export(src_path)

    orig_size = os.path.getsize(backup_path) / 1024
    new_size = os.path.getsize(src_path) / 1024
    print(f"Faces : {original_faces} → {len(decimated.faces)}")
    print(f"Size  : {orig_size:.1f} KB → {new_size:.1f} KB")
    print(f"Saved : {src_path}")


if __name__ == '__main__':
    main()