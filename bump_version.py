"""
bump_version.py — PlatformIO pre-script.
Автоматично піднімає PATCH-версію (X.Y.Z -> X.Y.Z+1) у Light_music.ino
ПЕРЕД компіляцією, щоб нова версія потрапила у сам бінарник.

Підключається лише в оточенні [env:release] у platformio.ini,
тому звичайний Build цього не чіпає.
"""

import re
from pathlib import Path

SKETCH_PATH = Path("src/Light_music.ino")


def bump_version():
    text = SKETCH_PATH.read_text(encoding="utf-8")
    match = re.search(r'#define\s+FIRMWARE_VERSION\s+"(\d+)\.(\d+)\.(\d+)"', text)
    if not match:
        print("[bump_version] FIRMWARE_VERSION не знайдено — пропускаю")
        return

    major, minor, patch = map(int, match.groups())
    new_version = f"{major}.{minor}.{patch + 1}"
    new_text = text[:match.start()] + f'#define FIRMWARE_VERSION "{new_version}"' + text[match.end():]
    SKETCH_PATH.write_text(new_text, encoding="utf-8")
    print(f"[bump_version] Версія піднята: {major}.{minor}.{patch} -> {new_version}")


bump_version()
