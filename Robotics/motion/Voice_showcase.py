import os
import time
import speech_recognition as sr
from faster_whisper import WhisperModel

# --- EXISTING REPO IMPORTS (NO CHANGES REQUIRED) ---
from dobot_driver import DobotDriver
from dh_gripper import DHGripperPGE
import robot_config as cfg

# --- EXTREMELY FORGIVING NOISE PARSER ---
def parse_voice(text):
    text = text.lower().strip()
    # As long as the keyword is inside whatever noise it picked up, it fires!
    if "start" in text or "begin" in text or "go" in text:
        return "START"
    if "drop" in text or "place" in text or "release" in text:
        return "DROP"
    return None

def main():
    print("==================================================")
    print("🚀 ARC 2026: LIVE VOICE SHOWCASE SCRIPT 🚀")
    print("==================================================")

    # 1. HARDWARE LINK (Uses exact config as your live repo)
    print("\n[HARDWARE] Connecting to Dobot & Gripper...")
    robot = DobotDriver() 
    try:
        robot.connect()
        robot.clear_and_enable(speed_percent=cfg.SPEED_TRAVEL)
        print("✅ DOBOT ARMED.")
    except Exception as e:
        print(f"❌ FATAL: Cannot connect to Robot. {e}")
        return

    gripper = DHGripperPGE(port=cfg.GRIPPER_PORT, baudrate=cfg.GRIPPER_BAUDRATE, device_id=cfg.GRIPPER_SLAVE_ID)
    try:
        gripper.connect()
        print("✅ GRIPPER LINKED.")
    except Exception as e:
        print(f"❌ FATAL: Cannot connect to Gripper. {e}")
        return

    # 2. VOICE AI INIT
    print("\n🧠 Loading Whisper AI (Tiny.en)...")
    model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    print("🎤 Shh for 2 seconds... calibrating to exhibition floor noise...")
    with mic as source: 
        recognizer.adjust_for_ambient_noise(source, duration=2.0)

    # 3. SET THE STAGE
    print("\n➡️ Moving to Home Position (NEUTRAL_3) & Opening Gripper...")
    robot.speed_factor(cfg.SPEED_TRAVEL)
    robot.movj_pose(cfg.NEUTRAL_3)
    gripper.open()
    robot.wait_until_idle()

    robot_has_block = False

    print("==================================================")
    print("✅ SYSTEM READY FOR DEMO!")
    print("   -> Say 'Start' to grab block P1.")
    print("   -> Say 'Drop'  to place it at T1.")
    print("==================================================")

    # 4. INFINITE LISTENING LOOP
    with mic as source:
        while True:
            try:
                state_text = "(Holding Block -> Awaiting 'Drop')" if robot_has_block else "(Empty -> Awaiting 'Start')"
                print(f"\n[MIC HOT] {state_text} Speak into the mic...")
                
                # phrase_time_limit cuts it off quickly so background crowd noise doesn't hang it!
                audio = recognizer.listen(source, timeout=10.0, phrase_time_limit=3.0)

                with open("demo_audio.wav", "wb") as f: 
                    f.write(audio.get_wav_data())
                    
                segments, _ = model.transcribe("demo_audio.wav", beam_size=1)
                text = " ".join([s.text for s in segments]).strip()
                
                if not text: continue
                
                print(f"🗣️ AI Transcribed: '{text}'")
                cmd = parse_voice(text)

                if not cmd:
                    print("🚫 (No keywords recognized, ignoring noise)")
                    continue

                # ---------------------------------------------
                # 💥 ACTION: "START"
                # ---------------------------------------------
                if cmd == "START" and not robot_has_block:
                    print("🚀 COMMAND: Auto-Pick P1 -> T1...")
                    
                    pick_target = cfg.PICKUP_POINTS["P1"]
                    hover_pose = (pick_target[0], pick_target[1], pick_target[2] + cfg.PICK_CLEARANCE_MM, pick_target[3], pick_target[4], pick_target[5])
                    
                    robot.speed_factor(cfg.SPEED_TRAVEL)
                    robot.movj_pose(hover_pose)
                    robot.wait_until_idle()

                    robot.speed_factor(cfg.SPEED_PRECISION)
                    robot.movl_pose(pick_target)
                    robot.wait_until_idle()

                    gripper.close()
                    time.sleep(0.5)

                    robot.movl_pose(hover_pose)
                    robot.wait_until_idle()

                    print("➡️ Carrying to Tower Hover (T1)...")
                    t1_hover = cfg.tower_hover_pose(level=0) 
                    robot.speed_factor(cfg.SPEED_TRAVEL)
                    robot.movj_pose(t1_hover)
                    robot.wait_until_idle()
                    
                    robot_has_block = True
                    print("🎯 Arrived. Awaiting 'Drop' command.")

                # ---------------------------------------------
                # 💥 ACTION: "DROP"
                # ---------------------------------------------
                elif cmd == "DROP" and robot_has_block:
                    print("🚀 COMMAND: Dropping at T1 & Returning Home...")
                    
                    t1_place = cfg.tower_place_pose(level=0)
                    t1_hover = cfg.tower_hover_pose(level=0)

                    robot.speed_factor(cfg.SPEED_PRECISION)
                    robot.movl_pose(t1_place)
                    robot.wait_until_idle()

                    gripper.open()
                    time.sleep(0.5)

                    robot.movl_pose(t1_hover)
                    robot.wait_until_idle()

                    print("➡️ Returning to NEUTRAL_3...")
                    robot.speed_factor(cfg.SPEED_TRAVEL)
                    robot.movj_pose(cfg.NEUTRAL_3)
                    robot.wait_until_idle()

                    robot_has_block = False
                    print("✅ Sequence Complete. Ready for next cycle.")

            except sr.WaitTimeoutError:
                pass # Perfectly normal timeout loop
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"⚠️ Warning: {e}")
                time.sleep(1)

    print("\nClosing hardware connections...")
    try: robot.close()
    except: pass
    try: gripper.close()
    except: pass
    print("Done.")

if __name__ == "__main__":
    main()