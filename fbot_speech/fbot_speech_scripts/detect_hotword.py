import sys
import os
import struct
import pyaudio
import pvporcupine

#access_key="Tbyk0dhsux2oYz/+GO8IGk05dCGmhTVze760CdDlA/vfLjkuGCqdRQ==" 
access_key = "IOI/v3Jkh0zwgEL5MEC2I0Dn/BVDZ8zfcQdFJhiYntRGFLf37F2gqw=="

class DetectHotWord():
    """
    @brief Class for detecting hotwords using Porcupine.
    This class initializes the Porcupine library with the specified hotword paths and sensitivities.
    It also handles audio input from the microphone and processes the audio frames to detect hotwords.
    """
    def __init__(self,
                keyword_path: list[str],
                sensitivity: float,
                library_path: str = None,
                model_path: str = None):
        """
        @brief Initialize the hotword detector.
        @param keyword_path: List of paths to the hotword models.
        @param sensitivity: Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A higher
        sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 will be used.
        @param library_path: Path to the Porcupine library (optional).
        @param model_path: Path to the Porcupine model (optional).
        """
        self.handle = pvporcupine.create(access_key=access_key, keyword_paths=keyword_path, sensitivities=sensitivity)
        self.mic = None

    def hear(self):
        """
        @brief Initialize the microphone for audio input.
        This function sets up the microphone stream for audio input using PyAudio.
        """
        self.pa = pyaudio.PyAudio()
        audio_stream = self.pa.open(
            rate=self.handle.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.handle.frame_length)
        self.mic = audio_stream

    def process(self):
        """
        @brief Process audio frames to detect hotwords.
        This function reads audio frames from the microphone and processes them using the Porcupine library.
        @return: Index of the detected hotword (0 for first hotword, 1 for second hotword, etc.).
        If no hotword is detected, it returns -1.
        """
        if self.mic is not None:
            pcm = self.mic.read(self.handle.frame_length)
            pcm = struct.unpack_from("h" * self.handle.frame_length, pcm)
            recorded_frames = []
            recorded_frames.append(pcm)
            result = self.handle.process(pcm)
            return result #Return an integer representing the index of the hotword detected, from zero.
        return -1

    def __del__(self):
        """
        @brief Clean up resources.
        This function closes the microphone stream and terminates the PyAudio instance.
        """
        self.mic.close()
        self.pa.terminate()
        self.handle.delete()

    