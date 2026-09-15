"""
auto_deploy.py — PlatformIO post-script.
Автоматично запускає deploy_ota.py ПІСЛЯ успішної компіляції release-збірки
(тобто тільки якщо build дійсно пройшов без помилок).

Підключається лише в оточенні [env:release] у platformio.ini.
"""

import subprocess
import sys

Import("env")  # noqa: F821 — це змінна, яку підставляє сам PlatformIO/SCons


def after_build(source, target, env):
    print("[auto_deploy] Build успішний — заливаю на Synology...")
    result = subprocess.run([sys.executable, "deploy_ota.py"])
    if result.returncode != 0:
        print("[auto_deploy] ПОМИЛКА під час заливки на Synology — перевір лог вище")
    else:
        print("[auto_deploy] Готово, версію залито на Synology")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build)
