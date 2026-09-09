# NAO robot notes — what works, what doesn't

Running log of findings about the physical robots and this codebase. Everything
here was **measured**, not assumed. Add to it as you learn more; the point is
that nobody has to rediscover the same things twice.

Last updated: 2026-09-09

---

## The robots

| BodyId | Version | State |
|---|---|---|
| `ALDR1312N090344` | V5.0 (naoH25) | **Walks, marginally.** Damaged/missing fingers. Needs `burst_gait`. |
| `ALDR1312N090346` | V5.0 (naoH25) | **Motors will not energise.** See below. |

They are visually identical — **put a physical sticker on one.** To check which
one you are on, run `whichnao.py`, or read
`Device/DeviceList/ChestBoard/BodyId`.

---

## Robot `…344` — the walking one

### The core fault

It cannot sustain a continuous walk. Roll oscillation alternates sign and
**roughly doubles every half second** while it is on one foot, until it topples.

Reproduced with **NAOqi's own default gait**, straight line, head held still, a
single `moveToward` call and none of this app's code:

```
t=0s 0.02  →  t=2s 0.16  →  t=3s 0.22  →  t=4s 0.35 rad → over
```

So it is the robot, not the software.

### Ruled out — with measurements. Don't re-litigate these.

| Suspect | Evidence it's fine |
|---|---|
| Servos / gearboxes | All joints track commanded angle to **<0.02 rad** under load |
| Foot FSRs | All 8 live; total **5.57 kg** standing (a NAO is ~5.4 kg) |
| IMU accuracy | Roll agrees with raw gravity to **0.04°** |
| IMU / bus comms | **0 NACKs** over 20 s on every device |
| Floor traction | Same behaviour on carpet as on office vinyl |
| Battery | 93–100%, healthy |
| Joint temperature | 28–44 °C, all status 0 |
| Autonomous Life | Disabled; not interfering |

Leading remaining theory is **mechanical play under dynamic load**, which no
software test can detect — position encoders read the motor side, so several
degrees of backlash reads as perfect tracking.

### Measured quirks

- **Stands ~9° forward-leaning.** `tiltY = +0.164 rad` in StandInit, confirmed
  against raw gravity. This eats most of its forward margin — hence the
  `torso_wy` trim.
- **Standing attitude varies by ~13° between runs** for an identical posture
  command (`tiltY` seen anywhere from −0.03 to +0.19). This is a large part of
  why behaviour looks random.
- **It is marginally stable.** Near that boundary, tiny differences in foot
  placement flip it between "six clean steps" and "falls on step one". Expect
  to shift the odds, not to achieve determinism.

---

## What works

### Gait — the configuration that finally walked well

Everything below aims at **one thing**: minimise time spent on a single foot,
because that is the only window in which roll can grow. Reacting faster does
not work — roll goes 0.13 → 0.47 in one second.

```json
"burst_gait": true, "step_m": 0.08, "torso_wy": -0.08
```

| Parameter | Value | Why |
|---|---|---|
| `MaxStepFrequency` | **1.000** | Faster steps = shorter single-support phase |
| `StepHeight` | **0.015** | Foot nearer the floor, less airborne time |
| `MaxStepX` | **0.020** | Smaller disturbance per stride |
| `MaxStepY` | **0.160** | Max legal lateral margin (min is 0.101!) |
| `TorsoWy` | **−0.08** | Trims the measured 9° forward lean |

### Burst gait — discrete `moveTo` steps

Walk one planned `moveTo(0.08m)`, then stand still until roll settles below
0.06, then repeat. Standing still is rock steady (roll sd 0.0003) and **damps
the oscillation**, so the build-up is reset before it can reach the point of no
return.

### Other things that work well

- **Standing still** — completely stable. All stationary features are unaffected
  by the balance problem.
- **Emergency Relax** — locks motion, kills queued tasks, releases joints.
- **Stuck detection via odometry** — `getRobotPosition(True)`.
- **Foot bumpers** — the only sensor that cannot miss an obstacle.
- **Speech, Gemini vision/chat, LEDs, head tracking** — all reliable.

---

## What does NOT work

### Robot behaviour

- **Continuous walking.** Roll diverges. Always.
- **Walking backward.** The least stable direction; it pitched backward and fell
  (`tiltY −0.37 → −0.94`) while roll was a healthy 0.03. Only reverse when
  genuinely wedged.
- **Getting up from lying down.** The routine pushes off the hands, and the
  fingers are damaged. It thrashes and risks more damage. The app now detects a
  lying posture and asks for help instead. **Sit it up by hand.**
- **Standing up from lying.** Same cause.

### Code patterns that actively caused falls

These are the traps. Every one of them was a real bug found the hard way.

- **`moveToward` in short timed bursts.** It has a ~1 s startup phase (weight
  shift, ZMP setup) before any foot leaves the ground. Cutting a burst at ~1 s
  stops it mid-preparation every cycle — the robot rocks its hips and manages
  ~2 real steps in 30 s. **Use `moveTo` for discrete moves.**
- **Interrupting a step mid-stride.** `stopMove()` while the robot is on one
  foot is the worst moment to freeze it. Roll routinely peaks 0.12–0.16 during a
  *normal* step and settles right back, so aborting on that number caused falls.
  Let the planned step finish — `moveTo` ends in stable double support.
- **Blocking the control loop.** A helper that polls `moveIsActive()` without
  also checking tilt leaves the robot unmonitored for seconds. It fell during
  exactly such a window.
- **Auto-recovering after a fall.** The old code ran `goToPosture("Sit")`
  straight after detecting a fall — a fresh full-body motion on a robot that had
  just hit the floor. It looked exactly like Relax being ignored. **Go limp and
  wait for a human.**
- **Commanding forward speed in the "shuffle band".** Anything below ~0.22
  normalised makes it perform a full weight-shift cycle for a millimetre of
  travel. Either walk properly or stop. Throttling speed toward zero *while
  still turning* toppled it outright.

### Sensor gotchas

- **Sonar is intermittent.** With an object genuinely at 0.5 m, readings flip
  between the true distance and `5.0` ("no echo") sample to sample. Refresh is
  only **~5 Hz**. Never trust a single reading — take the closest value in a
  time window.
- **Sonar is blind to a lot.** ~0.25 m minimum range, ~60° cone, chest-mounted.
  It misses low obstacles, thin ones (chair legs), soft ones (fabric absorbs
  ultrasound), and walls approached at an angle. **It will not see a wall your
  shoulder is touching.**
- **Sensor reads must fail safe.** A failed `getData` used to leave distance at
  "all clear" — a flaky link looked identical to an empty room.
- **`robotHasFallen` is latched.** It stays True until something clears it.
- **`ALSonar` must be subscribed** or the values silently stop refreshing, and a
  frozen reading is indistinguishable from clear space.

### NAOqi API traps

- **`moveToward` takes normalised velocities `[-1, 1]`, not m/s.** They are
  fractions of `MaxStepX`/`MaxStepTheta`. `0.05` is not "slow", it is ~1.5 mm of
  travel per step.
- **The frequency key is `MaxStepFrequency`, not `Frequency`.** The wrong
  spelling is silently ignored, so the setting appears to work and does nothing.
- **Out-of-range move config is not applied as written.** Check the robot's own
  `getMoveConfig("Min")`/`("Max")`. `MaxStepY` minimum is **0.101** — values
  below that were asking it to walk a tightrope.
- **`goToPosture` is a planner, not an animation.** It reads the current posture
  and searches a transition graph. A retry is never a replay — it starts from
  wherever the last attempt left the robot.
- **`getPostureFamily()` returns a *family***: `Standing`, `Sitting`, `Belly`,
  `Lying*`, `Unknown`. Posture *names* (`Stand`, `StandInit`) are never returned.
- **The classifier reports `Unknown` while settling.** Checking once right after
  `goToPosture` scores successful stands as failures. Poll for a few seconds.
- **`goToPosture` speed 1.0 is too fast.** Use 0.5–0.6. Max speed overshoots and
  trips the fall manager.
- **A resting robot has zero stiffness.** `goToPosture` silently does nothing.
  Call `wakeUp()` first.
- **`setSmartStiffnessEnabled`** — avoid during walking; a leg joint going soft
  mid-stride looks exactly like a forward collapse.
- **External collision protection `"All"` includes `Move`**, which halts the walk
  abruptly. An abrupt stop mid-stride is itself a fall risk. Use `"Arms"`.

---

## Robot `…346` — the dead one

Boots (chest LED pulses blue), reaches the network, then **will not energise its
motors at all**. `setStiffnesses("Body", 1.0)` is accepted without error and
every chain stays at 0.00 — head, both arms, both legs. Survives a clean reboot.
Battery 72%, joints thermally fine, chest button not stuck, Autonomous Life
disabled, no fault reported by any sensor.

Also boots and shuts down after ~1 minute on some attempts.

Next thing to try: **reseat the battery.** A pack that is inserted but not fully
latched fits all three symptoms — CPU runs off the charger, motors never get the
current they need, brownout under load.

---

## Environment / setup gotchas

- **The app must start with `PYTHONPATH` pointing at the vendored pynaoqi
  `lib/`**, or `naoqi` fails to import silently and you get
  *"TCP OK but ALProxy failed"*. `py27.py` only sets this when relaunching from
  Python 3 — running the bundled `Python\python.exe` directly skips it.
- **Run Python with `-u`.** Python 2 block-buffers stdout when redirected, so
  every diagnostic print sits unflushed and you debug blind.
- **The robot does not answer ICMP.** Ping sweeps will not find it; probe TCP
  9559 instead.
- **Don't scan with hundreds of parallel threads at a short timeout** — it
  produces false negatives and "loses" a robot that is right there.
- **Press the chest button once and it speaks its IP.** Fastest way to find it.
- **New router = it will not connect on its own.** Cable it to the router, then
  configure Wi-Fi at `http://<ip>/` (login `nao`/`nao`).

---

## Diagnostic scripts

In the session scratchpad. Worth moving into the repo if they are still useful.

| Script | Purpose |
|---|---|
| `whichnao.py` | Find every NAO on the subnet, identify by BodyId |
| `stiffness_test.py` | 5 s verdict on whether motors energise. **Run first.** |
| `walk_baseline.py` | 10 s verdict on walking, using stock NAOqi gait only |
| `servo_health.py` | FSRs, joint tracking error, current, temperature |
| `lean_check.py` | Static torso lean (this is where `torso_wy` came from) |
| `sonar_probe.py` | Sonar refresh rate, bumper keys, loop latency |
| `hw_faults.py` | Robot's self-reported faults and bus NACK counters |
| `launch_nao.ps1` | Start the app with a live log; closes the previous session |

---

## Open / untested

- **Wall-stuck escape** — just added, needs testing. Odometry detects a stalled
  step, then reverses 15 cm and turns 1.2–1.8 rad, alternating direction.
- **Corner trapping** — avoidance has no memory of directions already tried.
- **Deeper voice** — `\vct=70\` markup works today in the Speech Test box; not
  yet wired in as a persistent setting.
- **`torso_wy` value** — `-0.08` worked well. If it starts falling *backward*,
  reduce toward `-0.04`.
