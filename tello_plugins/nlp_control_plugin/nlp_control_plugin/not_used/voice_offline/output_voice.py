#!/usr/bin/env python3

from voice_text import transcribe_audio

# Specify the path to the Vosk model and the audio file
model_path = "vosk-model-small-en-us-0.15"  # Adjust this path to your model's location
audio_file_path = "audio_record/rec1.wav"  # Adjust this path to your audio file's location

# Call the transcribe_audio function
transcription = transcribe_audio(model_path, audio_file_path)

# Print or use the transcription result
if transcription:
    print("Transcription Result:")
    print(transcription)
else:
    print("Failed to transcribe audio.")
