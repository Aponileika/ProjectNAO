import socket
import time
from nao_app.config import _make_proxy


def powerable_joints(motion, memory):
    """Body joints whose position sensor reads inside the joint's limits.

    ALMotion refuses to stiffen a joint it believes is past its mechanical
    stop, and a single refusal makes setStiffnesses("Body", ...) apply to
    nothing at all.  On BodyId ALDR1312N090319 the RWristYaw sensor reads
    ~50 deg beyond its limit with the wrist physically centred, so every
    whole-body wake-up silently left the robot limp.  ALMotion.getAngles()
    clamps to the limits, so the raw ALMemory value is the only way to see it.

    The margin is wide on purpose.  A relaxed joint posed by hand sits a few
    hundredths past its stop (RHipPitch -1.59 against -1.535 in a hand-set
    crouch), and leaving a hip unpowered for that tips the robot on its face.
    A broken sensor is off by far more: RWristYaw reads 0.7-1.1 rad out."""
    names = motion.getBodyNames("Body")
    if memory is None:
        return names
    limits = motion.getLimits("Body")
    raw = memory.getListData(["Device/SubDeviceList/%s/Position/Sensor/Value" % n
                              for n in names])
    ok, out = [], []
    for n, lim, pos in zip(names, limits, raw):
        if pos is None or lim[0] - 0.3 <= pos <= lim[1] + 0.3:
            ok.append(n)
        else:
            out.append("%s=%+.2f" % (n, pos))
    if out:
        print("[Stiffness] left unpowered, sensor outside limits: %s" % ", ".join(out))
    return ok


def stiffen_body(motion, memory, value=1.0):
    joints = powerable_joints(motion, memory)
    motion.setStiffnesses(joints, value)
    if value <= 0:
        return
    # Read back only after the DCM has applied it: an immediate read returns
    # the old 0.0 and made healthy joints (HeadYaw, first in the list) look
    # refused.
    time.sleep(0.1)
    if max(motion.getStiffnesses(joints)) > 0:
        return
    # One refused joint voids the whole call.  Go joint by joint so only the
    # joint ALMotion actually refuses stays limp.
    refused = []
    for j in joints:
        motion.setStiffnesses(j, value)
        time.sleep(0.1)
        if motion.getStiffnesses(j)[0] <= 0:
            refused.append(j)
    print("[Stiffness] ALMotion refused: %s" % (", ".join(refused) or "none"))


def wake_up(motion, memory):
    """wakeUp() that works when one joint's sensor is out of range.

    Stiffening the usable joints first marks the robot awake, so wakeUp()
    then has nothing left to do and cannot trip over the bad joint."""
    stiffen_body(motion, memory, 1.0)
    motion.wakeUp()

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
        self.audio_recorder = None
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
        self.audio_recorder = _make_proxy("ALAudioRecorder", self.ip, self.port)
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
