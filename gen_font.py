import subprocess
import os
import urllib.request
import stat

Import("env")

FONT_DIR  = "font"
OUT_DIR   = "src"
BDF_URL   = "https://stncrn.github.io/u8g2-unifont-helper/unifont-13.0.06.bdf"
CONV_URL  = "https://raw.githubusercontent.com/olikraus/u8g2/refs/heads/master/tools/font/bdfconv/bdfconv.exe"
BDF_FILE  = os.path.join(FONT_DIR, "unifont-13.0.06.bdf")
CONV_FILE = os.path.join(FONT_DIR, "bdfconv.exe")
OUT_FILE  = os.path.join(OUT_DIR, "unifont_custom.c")

def download(url, dest):
    print(f"Downloading {os.path.basename(dest)}...")
    urllib.request.urlretrieve(url, dest)
    print(f"Downloaded: {dest}")

def generate_font():
    os.makedirs(FONT_DIR, exist_ok=True)

    if not os.path.exists(BDF_FILE):
        download(BDF_URL, BDF_FILE)

    if not os.path.exists(CONV_FILE):
        download(CONV_URL, CONV_FILE)
        os.chmod(CONV_FILE, os.stat(CONV_FILE).st_mode | stat.S_IEXEC)

    if not os.path.exists(OUT_FILE):
        print("Generating custom font...")
        subprocess.run([
            CONV_FILE, "-v", "-f", "1",
            "-m", "0-127,128-255,256-383,4352-4607,12288-12351,12352-12447,12448-12543,12592-12687,12688-12703,12736-12783,19968-28150,44032-55215",
            BDF_FILE, "-o", OUT_FILE, "-n", "unifont_custom"
        ], check=True)
        print(f"Font generated: {OUT_FILE}")
    else:
        print("Custom font already exists, skipping.")

generate_font()