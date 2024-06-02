#!/usr/bin/env python3

import argparse
import queue
import sys
import sounddevice as sd
import time
import json

q = queue.Queue()

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

def listen_for_activation_phrase(recognizer, activation_phrase="leo"):
    """Listen for the activation phrase."""
    print("Listening for activation phrase...")
    start_time = time.time()
    while time.time() - start_time < 20:  # Timeout after 20 seconds
        data = q.get()
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            if activation_phrase in result['text']:
                print("Activation phrase detected.")
                return True
        else:
            recognizer.PartialResult()
    return False

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    "-l", "--list-devices", action="store_true",
    help="show list of audio devices and exit")
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)
parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
    parents=[parser])
parser.add_argument(
    "-f", "--filename", type=str, metavar="FILENAME",
    help="audio file to store recording to")
parser.add_argument(
    "-d", "--device", type=int_or_str,
    help="input device (numeric ID or substring)")
parser.add_argument(
    "-r", "--samplerate", type=int, help="sampling rate")
parser.add_argument(
    "-m", "--model", type=str, help="language model; e.g. en-us, fr, nl; default is en-us")
args = parser.parse_args(remaining)

try:
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, "input")
        args.samplerate = int(device_info["default_samplerate"])

    with sd.RawInputStream(samplerate=args.samplerate, blocksize=8000, device=args.device,
            dtype="int16", channels=1, callback=callback):
        print("#" * 80)
        print("Press Ctrl+C to stop the recording")
        print("#" * 80)

        recognizer = KaldiRecognizer(model, args.samplerate)
        
        while True:
            if listen_for_activation_phrase(recognizer):
                print("Car activated. You can now give commands.")
                break

        partial_results = ""
        while True:
            data = q.get()
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                print("\nFinal Result:", result['text'])
                partial_results = ""
            else:
                partial_result = json.loads(recognizer.PartialResult())['partial']
                if partial_result != partial_results:
                    partial_results = partial_result
                    print("Partial result:", partial_results)

except KeyboardInterrupt:
    print("\nDone")
    parser.exit(0)
except Exception as e:
    parser.exit(type(e).__name__ + ": " + str(e))
