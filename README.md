# NAO Control Panel

A desktop control application for the SoftBank Robotics NAO humanoid robot, built with Python 2.7 and Tkinter.

## Features

- **Connection management** — connect/disconnect to NAO over the network via NAOqi SDK
- **Gamepad control** — drive NAO with a PS4/PS5 controller (via pygame), with tilt-based stability limiting
- **Camera feed** — live camera preview with face and people detection
- **Speech** — text-to-speech, language selection, and volume control
- **LEDs & Postures** — one-click LED colour control and posture presets (Stand, Sit, Crouch, etc.)
- **Gemini AI** — chat with NAO via Google Gemini (text or voice), with optional live camera vision input
- **Autonomous behaviours** — wander mode, face-seeking, quick command shortcuts
- **Battery readout** — live battery percentage

## Requirements

### Robot

- SoftBank Robotics NAO V5 or V6
- NAOqi 2.8.x firmware
- NAO and the PC on the same local network

### PC (Windows)

- The bundled **Python 2.7** interpreter at `Python/python.exe` is used at runtime (required by pynaoqi)
- The **pynaoqi SDK** is vendored at `pynaoqi-python2.7-.../lib/` — no separate install needed
- **pygame 1.9.6** — install into the bundled Python (see below)
- **Pillow** — install into the *host* Python 3 environment (used for JPEG conversion via subprocess)

Install bundled Python dependencies:

```
Python\python.exe -m pip install pygame==1.9.6
```

Install host Python dependencies (for JPEG offloading):

```
pip install Pillow
```

## Configuration

Edit `config.json` in the repo root to set the robot's IP address and port:

```json
{
    "ip": "192.168.x.x",
    "port": 9559,
    "filepath": "C:\\path\\to\\pynaoqi\\lib"
}
```

For the **Gemini API key**, create a `secrets.json` file in the repo root (this file is gitignored):

```json
{
    "gemini_key": "YOUR_GEMINI_API_KEY"
}
```

## Running

```
Python\python.exe nao_setup\nao_settings.py
```

Or with any Python version — `nao_settings.py` will automatically relaunch under the bundled Python 2.7:

```
python nao_setup\nao_settings.py
```

## Checking the Connection

```
python nao_setup\check_nao_connection.py 192.168.x.x
```

## Project Structure

```
nao_setup/
  nao_settings.py          Entry point
  py27.py                  Shared Python 2.7 auto-relaunch helper
  check_nao_connection.py  Standalone connection checker
  requirements.txt         Python 2.7 dependencies
  nao_app/
    main.py                App bootstrap and UI wiring
    config.py              NAOqi proxy helpers (_clamp, _lerp, _make_proxy)
    backend/
      nao_connection.py    NAO connection manager (ALProxy wrappers)
      controller.py        Gamepad controller (pygame)
      vision.py            Camera feed, face/people detection
    ai/
      gemini_client.py     Google Gemini API client (urllib2, Python 2.7)
    ui/
      app_window.py        Main Tkinter application window
      widgets.py           Shared UI helpers and colour constants

root_archive/              Legacy scripts and experiments (not actively used)

Python/                    Bundled CPython 2.7 (Windows)
pynaoqi-.../               Vendored NAOqi Python SDK
config.json                Local network config (not secret)
secrets.json               API keys (gitignored — do not commit)
```

## Notes

- The application **requires Python 2.7** at runtime because pynaoqi does not support Python 3
- `nao_settings.py` handles the Python version switch automatically
- `secrets.json` is gitignored; never put your Gemini API key in `config.json`
