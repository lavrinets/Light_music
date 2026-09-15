#!/usr/bin/env python3
"""
deploy_ota.py — заливає firmware.bin і version.json на Synology через WebDAV
після компіляції в PlatformIO.

Запуск (з кореня проєкту, ПІСЛЯ успішного Build у VS Code):
    python deploy_ota.py

Вимагає:
    pip install requests
"""

import re
import sys
import getpass
import requests
from pathlib import Path

# ======================= НАЛАШТУВАННЯ =======================
WEBDAV_HOST = "mystation.pp.ua"
WEBDAV_PORT = 5878
WEBDAV_USER = "admin"
WEBDAV_REMOTE_DIR = "/WEB/Light_music/firmware"   # шлях НА Synology (без хосту й порту) — WEB це справжня коренева спільна папка

# Публічна адреса, за якою САМА ПЛАТА скачає firmware.bin (Web Station, порт 85 — інший сервіс, ніж WebDAV!)
PUBLIC_DOWNLOAD_BASE = "https://mystation.pp.ua:85/Light_music/firmware"

def find_firmware_bin() -> Path:
    candidates = sorted(Path(".pio/build").glob("*/firmware.bin"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not candidates:
        sys.exit("Не знайдено жодного firmware.bin у .pio/build/*/ — спочатку зроби Build")
    return candidates[0]


SKETCH_PATH = Path("src/Light_music.ino")
PASSWORD_FILE = Path("webdav_password.txt")  # НЕ комітити на GitHub! додай у .gitignore
# ===============================================================


def get_password() -> str:
    if PASSWORD_FILE.exists():
        pwd = PASSWORD_FILE.read_text(encoding="utf-8").strip()
        if pwd:
            print(f"Пароль зчитано з {PASSWORD_FILE} (без запиту)")
            return pwd
    return getpass.getpass(f"Пароль WebDAV для {WEBDAV_USER}@{WEBDAV_HOST}: ")


def get_firmware_version() -> str:
    text = SKETCH_PATH.read_text(encoding="utf-8")
    match = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', text)
    if not match:
        sys.exit(f"Не знайшов FIRMWARE_VERSION у {SKETCH_PATH}")
    return match.group(1)


def webdav_put(session: requests.Session, base_url: str, remote_name: str, data: bytes):
    url = base_url.rstrip("/") + "/" + remote_name
    resp = session.put(url, data=data)
    if resp.status_code not in (200, 201, 204):
        sys.exit(f"Помилка заливки {remote_name}: HTTP {resp.status_code}\n{resp.text}")
    print(f"✓ Залито: {remote_name} ({len(data)} байт)")


def main():
    firmware_path = find_firmware_bin()
    print(f"Знайдено прошивку: {firmware_path}")

    version = get_firmware_version()
    print(f"Версія прошивки: {version}")

    password = get_password()

    base_url = f"https://{WEBDAV_HOST}:{WEBDAV_PORT}{WEBDAV_REMOTE_DIR}"

    session = requests.Session()
    session.auth = (WEBDAV_USER, password)
    session.verify = True  # постав False, якщо на Synology самопідписаний сертифікат (не Let's Encrypt)

    # 1. Заливаємо сам бінарник
    firmware_bytes = firmware_path.read_bytes()
    webdav_put(session, base_url, "firmware.bin", firmware_bytes)

    # 2. Генеруємо і заливаємо version.json
    version_json = (
        "{\n"
        f'  "version": "{version}",\n'
        f'  "url": "{PUBLIC_DOWNLOAD_BASE}/firmware.bin"\n'
        "}\n"
    ).encode("utf-8")
    webdav_put(session, base_url, "version.json", version_json)

    print("\nГотово! Плати самі підхоплять оновлення протягом години"
          " (або одразу після перезавантаження плати).")


if __name__ == "__main__":
    main()
