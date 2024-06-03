#!/usr/bin/env python3

import argparse
import json
import serial
import time
from vosk import Model, KaldiRecognizer

command_triggered = {
    1: False,  # Forward
    0: False,  # Backward
    2: False,  # Stop
    3: False,  # Left
    4: False   # Right
}

state = "stop"  # Initial state
previous_state = state  # Variable to track the previous state

cooldown_period = 4  # seconds

# Establish serial connection to Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

def handle_command(result_text):
    global state, command_triggered, last_action_time

    if "front" in result_text or "forward" in result_text:
        if not command_triggered[1]:
            ser.write(b'F')
            state = "running"
            last_action_time = time.time()
            command_triggered[1] = True
    elif "back" in result_text or "backward" in result_text:
        if not command_triggered[0]:
            ser.write(b'B')
            state = "running"
            last_action_time = time.time()
            command_triggered[0] = True
    elif "stop" in result_text:
        if not command_triggered[2]:
            ser.write(b'S')
            state = "stop"
            last_action_time = time.time()
            command_triggered[2] = True
    elif "lift" in result_text or "left" in result_text:
        if not command_triggered[3]:
            ser.write(b'L')
            state = "running"
            last_action_time = time.time()
            command_triggered[3] = True
    elif "right" in result_text:
        if not command_triggered[4]:
            ser.write(b'R')
            state = "running"
            last_action_time = time.time()
            command_triggered[4] = True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Command handling script")
    parser.add_argument("-r", "--result-text", type=str, help="Result text from the recognizer")
    args = parser.parse_args()
    
    handle_command(args.result_text)
