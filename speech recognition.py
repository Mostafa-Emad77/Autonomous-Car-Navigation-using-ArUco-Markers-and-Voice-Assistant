#!/usr/bin/env python3

import argparse
import json

from vosk import Model, KaldiRecognizer

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Speech recognition script")
    parser.add_argument("-s", "--samplerate", type=int, help="Sampling rate")
    parser.add_argument("-m", "--model", type=str, help="Language model; e.g. en-us, fr, nl; default is en-us")
    parser.add_argument("-w", "--waveform", type=str, help="Waveform data in bytes")
    parser.add_argument("-p", "--partial-result", type=str, help="Partial result from the recognizer")
    args = parser.parse_args()
    
    model = Model(lang=args.model if args.model else "en-us")
    
    if args.waveform:
        recognizer = KaldiRecognizer(model, args.samplerate)
        if recognizer.AcceptWaveform(bytes(args.waveform)):
            result = json.loads(recognizer.Result())
            print("\nFinal Result:", result['text'])
    elif args.partial_result:
        partial_result = json.loads(args.partial_result)['partial']
        print("Partial result:", partial_result)
