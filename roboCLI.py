#!/usr/bin/env python3

import re
import serial
import sys
import time
import tty
import termios
import threading

# Valid segment: F/B/R/L followed by a positive number, or standalone S
_SEGMENT_RE = re.compile(r'^[FBRLfbrl]\d+(\.\d+)?$|^[Ss]$')


def validate_track(raw):
    """Check that every comma-separated segment is a valid command.
    Returns (clean_segments_list, error_string_or_None)."""
    segments = [s.strip() for s in raw.split(",") if s.strip()]
    if not segments:
        return None, "empty track"
    for seg in segments:
        if not _SEGMENT_RE.match(seg):
            return None, f"invalid segment '{seg}'"
    return segments, None

PORTS = ["/dev/ttyACM1", "/dev/ttyACM2"]
BAUD = 115200

ser = None
selected_port = None

# Connect
for port in PORTS:
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        selected_port = port
        time.sleep(2)  # let Teensy reboot
        break
    except Exception:
        pass

if ser is None:
    print("No robot found")
    exit()

print(f"\n[CONNECTED] {selected_port}")
print("Controls:")
print("  x        = emergency stop  (instant)")
print("  q        = pause/resume    (instant)")
print("  r        = toggle RPM PID  (instant)")
print("  Ctrl+C   = exit CLI")
print("Track commands (type + Enter):")
print("  F1")
print("  R90")
print("  F1,R90,F2\n")


# Background serial monitor
def serial_reader():
    while True:
        try:
            line = ser.readline()
            if line:
                decoded = line.decode(errors="ignore").strip()
                if decoded:
                    print(f"\r[ROBOT] {decoded}")
        except Exception:
            break


threading.Thread(target=serial_reader, daemon=True).start()


def send_byte(ch):
    """Send a single raw byte (no newline) for instant commands."""
    ser.write(ch.encode())
    ser.flush()
    print(f"[SENT] {ch}")


def send_line(text):
    """Send a full line (with newline) for track commands."""
    ser.write((text + "\n").encode())
    ser.flush()
    print(f"[SENT] {text}")


def getch():
    """Read a single character from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def read_line_from_raw(first_char):
    """After a non-instant key, switch to cooked mode to read the rest of the line."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)  # restore cooked
        sys.stdout.write(first_char)  # echo the first char
        sys.stdout.flush()
        rest = input()  # read remaining (Enter to submit)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return first_char + rest


print("> ", end="", flush=True)

while True:
    try:
        ch = getch()

        # --- Instant commands (no Enter needed, no echo) ---
        if ch in ("x", "X"):
            print("x")
            send_byte("X")
            print("> ", end="", flush=True)
            continue

        if ch in ("q", "Q"):
            print("q")
            send_byte("Q")
            print("> ", end="", flush=True)
            continue

        if ch in ("r", "R"):
            print("r")
            send_byte("r")
            print("> ", end="", flush=True)
            continue

        # Ctrl+C or Ctrl+D → exit
        if ch in ("\x03", "\x04"):
            print()
            break

        # Enter with nothing typed
        if ch in ("\r", "\n"):
            print()
            print("> ", end="", flush=True)
            continue

        # --- Track command: read rest of line in cooked mode ---
        track = read_line_from_raw(ch).strip()

        if not track:
            print("> ", end="", flush=True)
            continue

        # Strip T: prefix if user typed it
        raw = track
        if raw.upper().startswith("T:"):
            raw = raw[2:]

        # Validate every segment before sending
        segments, err = validate_track(raw)
        if err:
            print(f"[ERROR] {err}  —  valid examples: F1, R90, B2.5, L45, S")
            print("> ", end="", flush=True)
            continue

        # Rebuild clean track (uppercase)
        clean = ",".join(s.upper() for s in segments)

        # Add ,S if missing
        if not clean.endswith(",S"):
            clean += ",S"

        send_line("T:" + clean)
        print("> ", end="", flush=True)

    except (KeyboardInterrupt, EOFError):
        print()
        break

print("\nClosing serial...")
ser.close()