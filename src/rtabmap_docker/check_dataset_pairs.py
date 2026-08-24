#!/usr/bin/env python3

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent / "datasets"
SEQUENCES = ROOT / "sequences"
GT = ROOT / "gt"


def stems_with_suffix(folder: Path, suffix: str):
    return {p.stem for p in folder.glob(f"*{suffix}") if p.is_file()}


def main():
    if not SEQUENCES.is_dir():
        print(f"[ERROR] Missing directory: {SEQUENCES}")
        return 2
    if not GT.is_dir():
        print(f"[ERROR] Missing directory: {GT}")
        return 2

    db3 = stems_with_suffix(SEQUENCES, ".db3")
    txt = stems_with_suffix(GT, ".txt")

    only_db3 = sorted(db3 - txt)
    only_txt = sorted(txt - db3)
    matched = sorted(db3 & txt)

    print(f"matched={len(matched)}")
    print(f"db3_only={len(only_db3)}")
    print(f"txt_only={len(only_txt)}")

    if matched:
        print("matched_names:")
        for name in matched:
            print(name)

    if only_db3:
        print("missing_txt_for:")
        for name in only_db3:
            print(name)

    if only_txt:
        print("missing_db3_for:")
        for name in only_txt:
            print(name)

    return 0 if not only_db3 and not only_txt else 1


if __name__ == "__main__":
    sys.exit(main())
