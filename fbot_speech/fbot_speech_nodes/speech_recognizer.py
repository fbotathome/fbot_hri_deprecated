#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import os
import threading
import time
from rclpy.node import Node

from fbot_speech_msgs.srv import SpeechToText
from ament_index_python.packages import get_package_share_directory
from playsound import playsound
from RealtimeSTT import AudioToTextRecorder

DEFAULT_LANGUAGE = 'en'


class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer')
        ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_behavior'), '../../../..'))
        self.pack_dir = os.path.join(ws_dir, "src", "fbot_hri", "fbot_speech",'audios')

        self.declareParameters()
        self.readParameters()
        self.initRosComm()

        self.recorder =  AudioToTextRecorder(**self.stt_configs)

        self.get_logger().info("Speech Recognizer is on!")

    def initRosComm(self):
        """
        @brief Initialize ROS communication for the node.
        This function sets up the publisher and subscriber for speech recognition.
        """
        # Create service
        self.speech_recognition_service = self.create_service(SpeechToText, self.recognizer_service_param, self.handleRecognition)

    def declareParameters(self):
        """
        @brief Declare parameters for the node.
        """
        self.declare_parameter('stt_configs.compute_type', 'float16')
        self.declare_parameter('stt_configs.spinner', False)
        self.declare_parameter('stt_configs.model', 'distil-medium.en')
        self.declare_parameter('stt_configs.silero_sensitivity', 0.8)
        self.declare_parameter('stt_configs.device', 'cpu')
        self.declare_parameter('stt_configs.webrtc_sensitivity', 1)
        self.declare_parameter('stt_configs.post_speech_silence_duration', 1.0)
        self.declare_parameter('stt_configs.min_length_of_recording', 3.0)
        self.declare_parameter('stt_configs.min_gap_between_recordings', 0)
        self.declare_parameter('stt_configs.enable_realtime_transcription', False)
        self.declare_parameter('stt_configs.silero_deactivity_detection', True)
        self.declare_parameter('stt_configs.initial_prompt', '')
        self.declare_parameter('stt_mic_timeout', 10)
        self.declare_parameter('services.speech_recognizer.service', '/fbot_speech/sr/speech_recognizer')

    def readParameters(self):
        """
        @brief Read parameters from the ROS parameter server.
        """
        self.recognizer_service_param = self.get_parameter('services.speech_recognizer.service').get_parameter_value().string_value
        self.stt_mic_timeout = self.get_parameter('stt_mic_timeout').get_parameter_value().integer_value
        self.stt_configs = {
            "spinner": self.get_parameter('stt_configs.spinner').get_parameter_value().bool_value,
            "model": self.get_parameter('stt_configs.model').get_parameter_value().string_value,
            "silero_sensitivity": self.get_parameter('stt_configs.silero_sensitivity').get_parameter_value().double_value,
            "device": self.get_parameter('stt_configs.device').get_parameter_value().string_value,
            "webrtc_sensitivity": self.get_parameter('stt_configs.webrtc_sensitivity').get_parameter_value().integer_value,
            "post_speech_silence_duration": self.get_parameter('stt_configs.post_speech_silence_duration').get_parameter_value().double_value,
            "min_length_of_recording": self.get_parameter('stt_configs.min_length_of_recording').get_parameter_value().double_value,
            "min_gap_between_recordings": self.get_parameter('stt_configs.min_gap_between_recordings').get_parameter_value().double_value,
            "enable_realtime_transcription": self.get_parameter('stt_configs.enable_realtime_transcription').get_parameter_value().bool_value,
            "silero_deactivity_detection": self.get_parameter('stt_configs.silero_deactivity_detection').get_parameter_value().bool_value,
            "initial_prompt": self.get_parameter('stt_configs.initial_prompt').get_parameter_value().string_value,
            "compute_type": self.get_parameter('stt_configs.compute_type').get_parameter_value().string_value,
        }

    def delayStarterRecorder(self):
        """
        @brief Delay the start of the recorder to allow for VAD detection."""
        time.sleep(0.5)
        playsound(self.pack_dir + '/beep.wav')

    def handleRecognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        self.get_logger().info("Handling recognition request...")

        # Variable to store the start time of VAD detection
        vad_start_time = [None]

        def check_vad_time(recorder):
            while True:
                seconds_pass = (time.time() - vad_start_time[0])
                if vad_start_time[0] is not None and seconds_pass > self.stt_mic_timeout or response.text != '':
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
        thread1 = threading.Thread(target=self.delayStarterRecorder)
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