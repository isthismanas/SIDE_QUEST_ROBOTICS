import os
import re
import math
import time
import speech_recognition as sr
from faster_whisper import WhisperModel

# The Exact Command Dictionary
GRAMMAR = {
    "DROP":["drop", "place", "commit", "release", "put it down"],
    "FIX":["fix", "adjust", "nudge mode"],
    "NUDGE_LEFT":["left", "move left", "nudge left"],
    "NUDGE_RIGHT":["right", "move right", "nudge right"],
    "NUDGE_FORWARD":["front", "forward", "move forward", "nudge forward", "up"],
    "NUDGE_BACK":["back", "backward", "move back", "nudge back", "down"]
}

def parse_command(text):
    # Strip punctuation and normalize
    text = re.sub(r'[^\w\s]', '', text.lower().strip())
    for cmd, words in GRAMMAR.items():
        for w in words:
            # \b matches word boundaries to avoid false triggers
            if re.search(r'\b' + re.escape(w) + r'\b', text):
                return cmd
    return None

def start_voice_assistant(cmd_callback, state_getter, threshold):
    """
    Runs continuously as a background daemon.
    Actively listens to the microphone but only triggers logic when the robot is waiting.
    """
    print("🧠 [VOICE] Loading offline Whisper AI (Tiny.en)...")
    model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
    recognizer = sr.Recognizer()
    
    try:
        mic = sr.Microphone()
    except Exception as e:
        print(f"❌ [VOICE] CRITICAL ERROR: No USB Microphone found. {e}")
        return
        
    print("🎤 [VOICE] Tuning to ambient room noise...")
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=2.0)
        
        print("✅ [VOICE] Linked with Task Controller. Microphone actively monitoring.")

        # Keep hardware source open constantly for zero-lag listen intervals
        while True:
            try:
                # 1. STATE GATING: Ask task_controller.py what the robot is doing
                current_state = state_getter()
                
                # ONLY turn on processing if robot is hovering (waiting for drop or nudge)
                if current_state not in ["WAITING_FOR_DECISION", "NUDGE"]:
                    time.sleep(0.5)
                    continue

                # 2. CAPTURE: 3 second blocks while at hover points
                # Phrase time limit prevents infinite hanging on background chatter
                print(f"\n👂 [VOICE ACTIVE] Robot State: {current_state}. Ready for command...")
                audio = recognizer.listen(source, timeout=5.0, phrase_time_limit=3.0)

                # Dump raw bytes directly for whisper analysis
                with open("runtime_audio.wav", "wb") as f:
                    f.write(audio.get_wav_data())

                segments_iter, _ = model.transcribe("runtime_audio.wav", beam_size=1)
                segments = list(segments_iter)
                
                if not segments: 
                    continue

                text = " ".join([s.text for s in segments]).strip()
                if not text: 
                    continue

                # 3. CONFIDENCE FALLBACK LOGIC
                # Whisper avg_logprob mapped to a standard percentage
                prob = math.exp(segments[0].avg_logprob)
                print(f"🗣️ User Said: '{text}' (Confidence: {int(prob * 100)}%)")

                if prob < threshold:
                    print("⚠️[VOICE FALLBACK] Whisper not confident enough.")
                    cmd_callback("LOW_CONFIDENCE")
                    time.sleep(2) # brief pause so we don't spam terminal
                    continue
                    
                # 4. COMMAND MAPPER
                valid_cmd = parse_command(text)
                if valid_cmd:
                    # Enforce the strict "MUST FIX BEFORE NUDGE" rule defined by Albert
                    if "NUDGE" in valid_cmd and current_state != "NUDGE":
                        print("🚫 [VOICE] Blocked: You must say 'Fix' before nudging.")
                        continue
                    
                    print(f"💥 [VOICE AUTHORIZED] Handing '{valid_cmd}' to Task Controller.")
                    cmd_callback(valid_cmd)
                else:
                    print("🚫[VOICE] Ignored: Word is not in active grammar.")

            except sr.WaitTimeoutError:
                # Silent timeout when no one speaks, naturally resets and re-evaluates STATE.
                pass
            except Exception as e:
                print(f"⚠️ [VOICE PIPELINE ERROR]: {e}")
                time.sleep(1)