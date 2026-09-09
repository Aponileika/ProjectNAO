# -*- coding: utf-8 -*-
"""Who the robot is, what it says, and how it sounds.

Everything about NAO's character lives here so the personality can be
rewritten without touching the control loops.  Three parts:

  * IDENTITY  - the facts the robot must never get wrong about itself.
  * PHRASES   - what it says while wandering, avoiding, celebrating, etc.
                Picked at random but without immediate repeats.
  * VoiceStyle- Acapela markup that shapes the voice (deeper, slower...).

Python 2.7 only, to match the rest of the app.
"""

import random


# ======================================================================
# IDENTITY
# ======================================================================

NAME         = "NAO"
MODEL        = "NAO V5 humanoid"
ORGANISATION = "FIA Robotics"
INSTITUTION  = "Linkoping University"
CITY         = "Linkoping"

# The English Acapela voice mangles Swedish spelling, so speech uses a
# phonetic form while text (Gemini prompts, the UI) uses the real one.
# Tweak SPOKEN_* if the pronunciation still sounds off on the robot.
SPOKEN_INSTITUTION = "Linshurping University"
SPOKEN_CITY        = "Linshurping"

# Traits the generated speech should hold to.  Kept short on purpose: long
# character briefs make a language model verbose, and this robot has to
# finish a sentence before it finishes a step.
TRAITS = [
    "warm and a bit wry, never sarcastic at anyone's expense",
    "proud of being a research robot, honest that walking is hard for him",
    "brief - one or two short sentences, spoken out loud, never written prose",
    "no emoji, no stage directions, no asterisks, plain speakable words only",
]


def identity_sentence(spoken=False):
    """One-line self description."""
    place = SPOKEN_INSTITUTION if spoken else INSTITUTION
    return "I am %s, a V5 humanoid from %s at %s." % (NAME, ORGANISATION, place)


def system_prompt(extra=""):
    """Character brief to prepend to every Gemini request.

    Anything the robot says out loud should go through this so it keeps one
    voice whether the words came from a phrase bank or from a model."""
    lines = [
        "You are %s, a %s belonging to %s at %s in %s, Sweden."
        % (NAME, MODEL, ORGANISATION, INSTITUTION, CITY),
        "You are a real physical robot standing in a lab, not an assistant "
        "in a chat window.  Everything you produce is read aloud immediately "
        "by your own text-to-speech, so write only speakable words.",
        "Character:",
    ]
    lines += ["  - " + t for t in TRAITS]
    if extra:
        lines.append(extra)
    return "\n".join(lines)


# ======================================================================
# PHRASE BANKS
# ======================================================================
#
# "%(target)s" is filled in with whatever is being searched for.  Lines
# without a placeholder work in any situation, which is why most of them
# have none - a blank-target walk should sound just as alive as a search.

PHRASES = {

    # Said once when a wander starts with no target: just going for a walk.
    "walk_start": [
        "Right, let us stretch these legs.",
        "Going for a walk. Back shortly.",
        "Time for a lap of the lab.",
        "Taking myself out for some air.",
        "Off I go. Mind the cables.",
    ],

    # Said once when a wander starts with a target.
    "search_start": [
        "Starting my search for the %(target)s.",
        "On the hunt for a %(target)s.",
        "Looking for the %(target)s. Give me a moment.",
        "Right then. Where is this %(target)s?",
    ],

    # Idle chatter while walking with no particular goal.  This is the bank
    # that plays most often, so it is the longest.
    "walking": [
        "Just stretching my legs.",
        "Nice day for a walk.",
        "Out for a stroll.",
        "Taking a look around.",
        "One step at a time.",
        "Every step is a small negotiation with gravity.",
        "I am %s, from %s. Walking is the hard part." % (NAME, ORGANISATION),
        "They built me at %s. They did not make it easy." % SPOKEN_INSTITUTION,
        "Balance is mostly confidence, I have decided.",
        "Left foot. Right foot. Repeat until interesting.",
        "This is going better than usual, and I have said too much.",
        "Somewhere in this building there is a flat piece of floor.",
        "I have twenty five motors and opinions about all of them.",
        "Do not mind me, I am just doing science.",
        "Slow is smooth. Smooth is upright.",
        "A robot walks into a lab. That is it, that is the walk.",
        "My hips are doing most of the thinking here.",
        "Still vertical. Small victories.",
        "Cruising. Sort of.",
        "I like this floor. We have an understanding.",
        "Just another day at %s." % SPOKEN_INSTITUTION,
        "If you see anything interesting, do say.",
    ],

    # Idle chatter while hunting a named object.
    "searching": [
        "Still looking for that %(target)s.",
        "No %(target)s yet.",
        "Where is that %(target)s hiding?",
        "Keep your eyes open, boss.",
        "Scanning the area.",
        "The %(target)s cannot hide forever.",
        "Sweeping the room for a %(target)s.",
        "My cameras say no %(target)s. My cameras say a lot of things.",
        "Give me a hint about this %(target)s. Anything.",
        "Search in progress. Morale high.",
    ],

    # Idle chatter while specifically looking for a person.
    "searching_human": [
        "Where is my human?",
        "I am a little lonely out here.",
        "Come out, come out, wherever you are.",
        "I know somebody is in this building.",
        "Show your face, I am friendly.",
        "Hello? Anyone want to meet a robot?",
        "I am %s from %s. I would love to say hello." % (NAME, ORGANISATION),
        "Human detected? Not yet. Hope remains.",
        "I promise I am nicer than I look.",
    ],

    # Something is in the way.
    "avoid_sonar": [
        "Whoa, blocked. Backing up.",
        "Something in front of me. Going around.",
        "That is a wall, or close enough.",
        "Nope. New direction.",
        "Blocked. Recalculating.",
        "I will take the scenic route.",
    ],
    "avoid_bumper": [
        "Oops, I bumped something. Backing up.",
        "Sorry. That was me.",
        "Found it with my foot. Classic.",
        "That was solid. Noted.",
    ],
    "avoid_boundary": [
        "Whoa, leaving the floor. Backing up.",
        "That is the edge. Not today.",
        "Floor ends here. Turning back.",
    ],

    # Wedged and getting out of it.
    "stuck": [
        "I am stuck. Backing out.",
        "This corner and I are done. Reversing.",
        "Wedged. Give me a second.",
        "Walking on the spot is not walking. Backing out.",
    ],

    # Remembered somewhere that trapped it before.
    "remembered_block": [
        "I have been stuck here before. Going another way.",
        "I remember this spot. Not falling for it twice.",
        "Bad memories here. Turning.",
    ],

    # Found what it was looking for.
    "found_human": [
        "There you are! I found a human!",
        "A person! Hello! Mission complete!",
        "Found you! I knew somebody was here!",
        "Human located. This is the best part of my day.",
    ],
    "found_target": [
        "I found the %(target)s! Mission complete!",
        "There it is! One %(target)s, located!",
        "Got it. That is the %(target)s.",
        "%(target)s found. I would take a bow, but balance.",
    ],

    # Went over.
    "fell": [
        "I fell. I have relaxed my motors. Please help me up.",
        "Down I go. Motors released. A hand, please?",
        "That did not work. I am limp and safe. Please pick me up.",
    ],

    # Task finished, sitting down.
    "task_done": [
        "That is me done. Sitting down.",
        "Task complete. Taking a seat.",
        "Finished. I will rest here.",
    ],

    # Said on connect.
    "greeting": [
        "Connected. %s here." % identity_sentence(spoken=True),
        "Hello. I am %s, from %s at %s."
        % (NAME, ORGANISATION, SPOKEN_INSTITUTION),
        "Online and upright. %s, ready." % NAME,
        "Good to be back. %s from %s reporting."
        % (NAME, SPOKEN_INSTITUTION),
    ],
}


class PhrasePicker(object):
    """Random phrases that do not repeat until the bank has been used up.

    Plain random.choice repeats itself often enough to sound broken - across
    a five line bank you hear the same sentence twice in a row roughly one
    time in five.  This shuffles each bank and walks through it instead."""

    def __init__(self, banks=None):
        self._banks = banks if banks is not None else PHRASES
        self._queues = {}

    def line(self, category, **fmt):
        bank = self._banks.get(category)
        if not bank:
            return ""
        queue = self._queues.get(category)
        if not queue:
            queue = list(bank)
            random.shuffle(queue)
            # Avoid starting the new pass with whatever ended the last one.
            last = getattr(self, "_last_" + category, None)
            if last is not None and len(queue) > 1 and queue[0] == last:
                queue.append(queue.pop(0))
            self._queues[category] = queue
        text = queue.pop(0)
        setattr(self, "_last_" + category, text)
        if fmt:
            try:
                text = text % fmt
            except (KeyError, ValueError, TypeError):
                pass
        return text


# ======================================================================
# VOICE
# ======================================================================
#
# Two independent knobs on NAO's Acapela engine:
#
#   \vct=N\   vocal tract length, 50-200, 100 = default.  This is the one
#             that matters.  It models the size of the speaker's head and
#             throat, so lowering it does not just pitch-shift a child
#             voice down - it makes the voice sound like it came out of a
#             bigger body.  Around 75-85 reads as an adult man.
#   \rspd=N\  relative speed, 50-200, 100 = default.  Slowing slightly
#             reads as heavier and more deliberate; too slow sounds drunk.
#
# ALTextToSpeech.setParameter("pitchShift", x) is deliberately not used:
# measured on this robot, 0.8 is rejected outright while 1.0 and 1.2 are
# accepted, so it can raise a voice but never lower one.  Markup is also
# per-utterance, which means the Speech Test box and the wander loop cannot
# fight over a global setting.
#
# Nor is setVoice an option here: this robot has only 'naoenu' and
# 'Emma22Enhanced' installed, both child/female.  Vocal tract length is the
# only route to a masculine voice without installing another Acapela voice.

VCT_MIN,  VCT_MAX  = 50, 150
RSPD_MIN, RSPD_MAX = 60, 140

VOICE_PRESETS = [
    # label,        vct, rspd
    ("Stock NAO",   100, 100),
    ("Grown up",     88,  96),
    ("Guy",          80,  92),
    ("Deep guy",     72,  90),
    ("Very deep",    62,  86),
]

DEFAULT_PRESET = "Deep guy"


class VoiceStyle(object):
    """Wraps text in the Acapela markup for the configured voice."""

    def __init__(self, vct=None, rspd=None):
        presets = dict((name, (v, r)) for name, v, r in VOICE_PRESETS)
        d_vct, d_rspd = presets[DEFAULT_PRESET]
        self.vct  = int(vct)  if vct  is not None else d_vct
        self.rspd = int(rspd) if rspd is not None else d_rspd

    def clamp(self):
        self.vct  = max(VCT_MIN,  min(VCT_MAX,  int(self.vct)))
        self.rspd = max(RSPD_MIN, min(RSPD_MAX, int(self.rspd)))

    def apply(self, text):
        """Prefix `text` with the voice markup, unless it already has some.

        Text typed into the Speech Test box may carry its own markup for
        experimenting; leave that alone."""
        if text is None:
            return ""
        text = str(text)
        if "\\vct=" in text or "\\rspd=" in text:
            return text
        self.clamp()
        if self.vct == 100 and self.rspd == 100:
            return text
        return "\\vct=%d\\ \\rspd=%d\\ %s" % (self.vct, self.rspd, text)

    def as_dict(self):
        self.clamp()
        return {"voice_vct": self.vct, "voice_rspd": self.rspd}
