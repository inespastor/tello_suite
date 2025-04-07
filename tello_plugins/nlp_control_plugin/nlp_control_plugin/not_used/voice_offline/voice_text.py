#!/usr/bin/env python3

import os
import sys
import wave
import logging
from vosk import Model, KaldiRecognizer

# Set up basic configuration for logging
logging.basicConfig(level=logging.WARNING)  # This will suppress informational messages

def transcribe_audio(model_path, audio_file_path):
    try:
        model = Model(model_path)
        wf = wave.open(audio_file_path, "rb")

        if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
            print("Audio file must be WAV format.")
            return None

        recognizer = KaldiRecognizer(model, wf.getframerate())
        final_result = ""

        while True:
            data = wf.readframes(4000)
            if len(data) == 0:
                break
            if recognizer.AcceptWaveform(data):
                final_result += recognizer.Result()
        
        final_result += recognizer.FinalResult()
        return final_result

    except Exception as e:
        print("An error occurred:", str(e))
        return None

    finally:
        wf.close()

if __name__ == "__main__":
    model_path = "vosk-model-small-en-us-0.15"
    audio_file_path = "audio_record/rec1.wav"
    result = transcribe_audio(model_path, audio_file_path)
    if result:
        print("Transcription Result:")
        print(result)
