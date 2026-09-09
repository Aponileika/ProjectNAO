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

**Mind the version gap: the robots run NAOqi `2.1.4.13`, while the desktop SDK
is `pynaoqi 2.8.6.23`.** They interoperate fine, but the API surface you get is
the robot's 2.1, not the 2.8 the SDK folder name suggests — so check behaviour
against the robot rather than trusting 2.8 documentation. Verified with
`ALMotion.getRobotConfig()` and `ALSystem.systemVersion()` on `…344`
(`Model Type naoH25`, `Head/Body/Arm Version VERSION_50`,
`RobotConfig/Body/BaseVersion V5.0`, head `ALDT1312N090363`).

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

## Run log

Newest first. Record what changed and what actually happened.

### 2026-09-09 — first genuinely good walk

Config: `burst_gait` on, `step_m` 0.08, `torso_wy` -0.08, `MaxStepFrequency`
1.0, `StepHeight` 0.015, `MaxStepX` 0.020, `MaxStepY` 0.160.

Walked well and at length — the first run not dominated by falling. The
`TorsoWy` trim plus the shorter single-support phase is the combination that
made the difference.

Remaining problem: **wall-stuck escape kept returning to the same spot.** It
escaped, wandered, and walked back into the same corner repeatedly, because the
escape turn was random and nothing remembered where it had been stopped. Led to
the spatial memory below.

### Earlier

Roll divergence dominated everything. See the fault section above.

---

## Spatial memory (added 2026-09-09, untested)

The robot now remembers where it was blocked and steers away from those places.

- Odometry (`getRobotPosition(True)`) supplies x/y/theta in a session-long frame.
- Every stuck event or sonar/bumper avoidance records a point; nearby hits merge
  rather than accumulating duplicates.
- Escapes and avoidance turns score candidate headings by how much remembered
  blockage lies ahead, and pick the clearest.
- Points expire after **180 s**, and the whole map is **discarded on a fall**.

**Storage is not the limit** — this runs on the PC, so thousands of points would
be fine. **Odometry drift is the limit.** Leg odometry accumulates error over
minutes, and a fall (plus being picked up and put down) destroys the reference
completely. Hence the TTL and the discard-on-fall: better no map than a wrong
one. Don't be tempted to raise the TTL much without checking drift first.

---

## UI

- **View menu** — Volume, Language, LEDs, Speech & Voice, PS5 Controller and
  Camera are hidden by default and toggled from `View`. They are rarely needed
  and crowded the window. Toggling repacks the whole column so cards keep their
  original order. To hide another card, add its exact title to
  `OPTIONAL_CARDS`; the title is read back off the `LabelFrame`, so a typo
  fails silently by simply leaving the card visible.
- **Map window** (`View → Map window`) — live top-down view at ~2.5 fps: green
  arrow for the robot and its heading, blue trail, red circles for remembered
  blocked points drawn at the real 0.45 m avoidance radius, 1 m grid. Drawn in
  the odometry frame, so it only means anything within a single walk.
- **Language is forced to English on connect** rather than read from the robot.
  With the card hidden you would not notice it had come up in another language
  until it started talking.

---

## Personality and voice (added 2026-09-10)

All of it lives in `nao_app/backend/persona.py`, so the character can be
rewritten without touching a control loop.

- **Identity** — the robot is NAO, a V5 humanoid belonging to **FIA Robotics at
  Linköping University**. `persona.system_prompt()` prepends that, plus the
  character traits, to every Gemini request whose answer gets spoken
  (`NaoAppWindow._in_character`). Deliberately *not* applied to the wander
  loop's yes/no vision classifier — personality there corrupts answer parsing.
- **Spoken spelling** — the English Acapela voice mangles "Linköping", so
  `SPOKEN_INSTITUTION = "Linshurping University"` is used in speech while the
  real spelling goes into text and prompts. Adjust that string if it still
  sounds wrong out loud.
- **Phrase banks** — `PHRASES` is keyed by situation (`walking`, `searching`,
  `searching_human`, `avoid_sonar`, `avoid_bumper`, `avoid_boundary`, `stuck`,
  `found_human`, `found_target`, `fell`, `greeting`, …). `PhrasePicker` shuffles
  each bank and walks through it rather than using `random.choice`, which in a
  five-line bank repeats itself back-to-back about one time in five and sounds
  broken. The idle `walking` bank is the longest (22 lines) because it plays
  most often.
- **One speech path** — `NaoAppWindow.say()` / `.say_line()` and
  `VisionManager.say()` are the only places that call `tts.say`. Everything is
  styled and nothing raises: a TTS failure must never take down a control loop.

### Making the voice deeper

- **`\vct=N\` (vocal tract length, 50–150, 100 = stock) is the knob that
  matters.** It models the size of the speaker's head and throat, so lowering it
  does not merely pitch-shift a child voice down — it sounds like it came out of
  a bigger body. ~80 reads as an adult man, ~72 noticeably deeper.
- `\rspd=N\` (relative speed, 60–140) slows it slightly, which reads as heavier
  and more deliberate. Too slow just sounds drunk.
- **`setParameter("pitchShift", x)` is useless for this** — measured on `…344`:
  `0.8` is rejected, `1.0` and `1.2` accepted. It can raise a voice, never lower
  one. Markup also applies per utterance, so the Speech Test box and the wander
  loop cannot fight over a global setting.
- **`setVoice` is not an option either** — only two voices are installed,
  `naoenu` (default) and `Emma22Enhanced`, both child/female. A voice dropdown
  would have nothing masculine to offer. Vocal tract length is the only route
  without installing another Acapela voice on the robot.
- Presets in the `Speech & Voice` card: Stock NAO / Grown up / Guy / Deep guy /
  Very deep, plus Depth and Speed sliders. **Save** writes `voice_vct` and
  `voice_rspd` into `config.json`; currently `72 / 90` ("Deep guy").
- Text that already contains its own `\vct=` or `\rspd=` passes through
  unstyled, so the Speech Test box still works for experimenting.

---

## Open / untested

- **Spatial memory and map window** — just added; watch the `[Map]` log lines to
  see whether chosen headings actually avoid known-blocked places.
- **Voice depth on the actual speaker** — `72 / 90` is a starting guess. NAO's
  small speaker rolls off low frequencies, so very low `vct` may sound thin
  rather than deep; tune by ear with the card's Test button.
- **`torso_wy` value** — `-0.08` worked well. If it starts falling *backward*,
  reduce toward `-0.04`.
- **Longer-horizon mapping** would need drift correction (landmarks, or resetting
  the frame against a known feature). Not worth it unless walks get much longer.
