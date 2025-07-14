#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import SpeechToText
import os
import threading
import time
from playsound import playsound

from RealtimeSTT import AudioToTextRecorder

DEFAULT_LANGUAGE = 'en'
PACK_DIR = os.path.join(os.path.expanduser("~"), 'fbot_ws', 'src', 'fbot_hri', 'fbot_speech')
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")


class SpeechRecognizerNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name=node_name)
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        
        self.recorder =  AudioToTextRecorder(**self.configs)

        self.get_logger().info("Speech Recognizer is on!")

    def initRosComm(self):
        self.speech_recognition_service = self.create_service(SpeechToText, self.recognizer_service_param, self.handleRecognition)
        self.audio_player_beep_service = self.create_client(Empty, self.audio_player_beep_param_service)

    def declareParameters(self):
        self.declare_parameter('stt_configs.compute_type', 'float32')
        self.declare_parameter('stt_configs.spinner', False)
        self.declare_parameter('stt_configs.model', 'small.en')
        self.declare_parameter('stt_configs.silero_sensitivity', 0.7)
        self.declare_parameter('stt_configs.device', 'cpu')
        self.declare_parameter('stt_configs.webrtc_sensitivity', 1)
        self.declare_parameter('stt_configs.post_speech_silence_duration', 1.0)
        self.declare_parameter('stt_configs.min_length_of_recording', 2.0)
        self.declare_parameter('stt_configs.min_gap_between_recordings', 0.0)
        self.declare_parameter('stt_configs.enable_realtime_transcription', False)
        self.declare_parameter('stt_configs.silero_deactivity_detection', True)
        self.declare_parameter('stt_configs.initial_prompt', '')
        self.declare_parameter('stt_mic_timeout', 10)
        self.declare_parameter('services.audio_player_beep.service', '/fbot_speech/ap/audio_beep')
        self.declare_parameter('services.speech_recognizer.service', '/fbot_speech/sr/speech_recognizer')


    def readParameters(self):
        self.audio_player_beep_param_service = self.get_parameter('services.audio_player_beep.service').get_parameter_value().string_value
        self.recognizer_service_param = self.get_parameter('services.speech_recognizer.service').get_parameter_value().string_value
        self.stt_mic_timeout = self.get_parameter('stt_mic_timeout').get_parameter_value().integer_value
        self.configs={
            'compute_type': self.get_parameter('stt_configs.compute_type').get_parameter_value().string_value,
            'spinner': self.get_parameter('stt_configs.spinner').get_parameter_value().bool_value,
            'model': self.get_parameter('stt_configs.model').get_parameter_value().string_value,
            'silero_sensitivity': self.get_parameter('stt_configs.silero_sensitivity').get_parameter_value().double_value,
            'device': self.get_parameter('stt_configs.device').get_parameter_value().string_value,
            'webrtc_sensitivity': self.get_parameter('stt_configs.webrtc_sensitivity').get_parameter_value().integer_value,
            'post_speech_silence_duration': self.get_parameter('stt_configs.post_speech_silence_duration').get_parameter_value().double_value,
            'min_length_of_recording': self.get_parameter('stt_configs.min_length_of_recording').get_parameter_value().double_value,
            'min_gap_between_recordings': self.get_parameter('stt_configs.min_gap_between_recordings').get_parameter_value().double_value,
            'enable_realtime_transcription': self.get_parameter('stt_configs.enable_realtime_transcription').get_parameter_value().bool_value,
            'silero_deactivity_detection': self.get_parameter('stt_configs.silero_deactivity_detection').get_parameter_value().bool_value,
            'initial_prompt': self.get_parameter('stt_configs.initial_prompt').get_parameter_value().string_value,

        }


    def delayStarterRecorder(self):
        time.sleep(0.5)
        self.audio_player_beep_service.call(Empty.Request())
    
    def timeout(self):
        init_time = (self.get_clock().now()).nanoseconds / 1e9
        while (self.stt_mic_timeout + init_time) > ((self.get_clock().now()).nanoseconds/1e9):
            continue
        self.get_logger().info("Timeout reached, stopping the recorder...")
        self.recorder.stop()
        self.recorder.abort()
            

    def handleRecognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        text = ''
        self.get_logger().info("Handling recognition request...")

        timeout = threading.Thread(target=self.timeout)
        timeout.start()

        # Start delayed starter recorder thread
        delay_starter = threading.Thread(target=self.delayStarterRecorder)
        delay_starter.start()
        self.get_logger().info("Start recoder")

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
        self.get_logger().info(f"Response: {text}")
        response.text = text
        return response


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    speech_recognizer_node = SpeechRecognizerNode(node_name = 'speech_recognizer')

    # Spin the node to keep it running
    rclpy.spin(speech_recognizer_node)

    # Clean up before shutting down
    speech_recognizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()