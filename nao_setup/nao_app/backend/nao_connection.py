import socket
from nao_app.config import _make_proxy

class NaoConnectionManager(object):
    def __init__(self):
        self.ip = None
        self.port = 9559
        self.connected = False
        
        self.system = None
        self.tts = None
        self.leds = None
        self.motion = None
        self.posture = None
        self.memory = None
        self.life = None
        self.audio = None
        self.video = None
        self.face = None
        self.people = None
        
    def connect(self, ip, port=9559):
        self.ip = ip
        self.port = port
        
        # TCP check
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3)
        try:
            s.connect((self.ip, self.port))
            s.close()
        except Exception as e:
            self.connected = False
            return False, "TCP port {} unreachable on {}: {}".format(self.port, self.ip, str(e))
            
        # Build proxies
        self.system = _make_proxy("ALSystem", self.ip, self.port)
        if self.system is None:
            self.connected = False
            return False, "TCP OK but ALProxy failed"
            
        self.tts = _make_proxy("ALTextToSpeech", self.ip, self.port)
        self.leds = _make_proxy("ALLeds", self.ip, self.port)
        self.motion = _make_proxy("ALMotion", self.ip, self.port)
        self.posture = _make_proxy("ALRobotPosture", self.ip, self.port)
        self.memory = _make_proxy("ALMemory", self.ip, self.port)
        self.life = _make_proxy("ALAutonomousLife", self.ip, self.port)
        self.audio = _make_proxy("ALAudioDevice", self.ip, self.port)
        self.video = _make_proxy("ALVideoDevice", self.ip, self.port)
        self.face = _make_proxy("ALFaceDetection", self.ip, self.port)
        self.people = _make_proxy("ALPeoplePerception", self.ip, self.port)
        
        self.connected = True
        
        ver = "?"
        if self.system:
            try:
                ver = self.system.systemVersion()
            except Exception:
                pass
                
        return True, "Connected to {} ({})".format(self.ip, ver)

    def get_proxy(self, name):
        # Lazy initialization
        if not self.connected:
            return None
        proxy_map = {
            "ALTextToSpeech": self.tts,
            "ALLeds": self.leds,
            "ALMotion": self.motion,
            "ALRobotPosture": self.posture,
            "ALMemory": self.memory,
            "ALAutonomousLife": self.life,
            "ALAudioDevice": self.audio,
            "ALVideoDevice": self.video,
            "ALFaceDetection": self.face,
            "ALPeoplePerception": self.people,
            "ALSystem": self.system
        }
        if name in proxy_map and proxy_map[name] is not None:
            return proxy_map[name]
        
        p = _make_proxy(name, self.ip, self.port)
        return p
