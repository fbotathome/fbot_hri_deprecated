#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import SpeechToText, SpeechToTextResponse
import os
import threading
import time
from playsound import playsound

from RealtimeSTT import AudioToTextRecorder

DEFAULT_LANGUAGE = 'en'
PACK_DIR = os.path.join(os.path.expanduser("~"), 'ros_workspace', 'src', 'fbot_speech')
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")


class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer')

        # Declare parameters
        self.declare_parameter("stt_configs", {
            "spinner": False,
            "model": "distil-medium.en",
            "silero_sensitivity": 0.8,
            "device": "cpu",
            "webrtc_sensitivity": 0.6,
            "post_speech_silence_duration": 1.0,
            "min_length_of_recording": 3,
            "min_gap_between_recordings": 0,
            "enable_realtime_transcription": False,
            "silero_deactivity_detection": True,
        })
        self.declare_parameter("stt_mic_timeout", 10)
        self.declare_parameter("services/speech_recognizer/service", "/fbot_speech/sr/speech_recognizer")

        # Fetch configurations
        self.configs = self.get_parameter("stt_configs").get_parameter_value().string_value
        self.stt_mic_timeout = self.get_parameter("stt_mic_timeout").get_parameter_value().integer_value

        # Setup the service
        recognizer_service_param = self.get_parameter("services/speech_recognizer/service").get_parameter_value().string_value
        self.speech_recognition_service = self.create_service(SpeechToText, recognizer_service_param, self.handle_recognition)

        # Global recorder variable
        self.recorder =  AudioToTextRecorder(**self.configs)

        self.get_logger().info("Speech Recognizer is on!")

    def delay_starter_recorder(self):
        time.sleep(0.5)
        playsound(TALK_AUDIO)

    def handle_recognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        self.get_logger().info("Handling recognition request...")

        # Variable to store the start time of VAD detection
        vad_start_time = [None]

        def check_vad_time(recorder):
            while True:
                seconds_pass = (time.time() - vad_start_time[0])
                if vad_start_time[0] is not None and seconds_pass > self.stt_mic_timeout:
                    self.get_logger().info(f"Stopping listening, too long... {seconds_pass:.1f}s")
                    recorder.stop()
                    recorder.abort()
                    break
                time.sleep(0.1)

        # Start VAD checking thread
        vad_start_time.__setitem__(0, time.time())
        vad_thread = threading.Thread(target=check_vad_time, args=(self.recorder,))
        vad_thread.start()

        # Start delayed starter recorder thread
        thread1 = threading.Thread(target=self.delay_starter_recorder)
        thread1.start()

        # Get recognized text
        try:
            text = self.recorder.text().lower()
        except Exception as e:
            self.get_logger().error(f"Error in recognition: {e}")
            text = ''

        # Shutdown the recorder
        try:
            self.recorder.stop()
        except:
            pass

        # Return the recognized text
        response.text = text
        return response


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    speech_recognizer_node = SpeechRecognizerNode()

    # Spin the node to keep it running
    rclpy.spin(speech_recognizer_node)

    # Clean up before shutting down
    speech_recognizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
