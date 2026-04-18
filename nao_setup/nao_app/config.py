# config.py
# Utility functions extracted from nao_settings.py

ALProxy = None
try:
    from naoqi import ALProxy as _ALProxy
    ALProxy = _ALProxy
except ImportError:
    pass

def _make_proxy(name, ip, port):
    if ALProxy is None:
        return None
    try:
        return ALProxy(name, str(ip), int(port))
    except Exception:
        return None

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _lerp(current, target, factor):
    return current + (target - current) * factor