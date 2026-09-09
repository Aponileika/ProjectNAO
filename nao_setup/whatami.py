import sys
from naoqi import ALProxy
ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.200"
try:
    m = ALProxy("ALMotion", ip, 9559)
except Exception as e:
    print "cannot reach %s: %s" % (ip, e)
    sys.exit(1)
print "--- ALMotion.getRobotConfig() ---"
cfg = m.getRobotConfig()
for k, v in zip(cfg[0], cfg[1]):
    print "%-28s %s" % (k, v)
try:
    mem = ALProxy("ALMemory", ip, 9559)
    for key in ("Device/DeviceList/ChestBoard/BodyId",
                "RobotConfig/Body/Type",
                "RobotConfig/Body/BaseVersion",
                "RobotConfig/Head/FullHeadId"):
        try:    print "%-40s %s" % (key, mem.getData(key))
        except Exception as e: print "%-40s (n/a)" % key
except Exception as e:
    print "ALMemory: %s" % e
try:
    s = ALProxy("ALSystem", ip, 9559)
    print "robot name: %s   naoqi: %s" % (s.robotName(), s.systemVersion())
except Exception as e:
    print "ALSystem: %s" % e
