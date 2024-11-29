#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError
from playsound import playsound
import os
import rospkg
import numpy as np

from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC, pipeline

from termcolor import colored
import warnings

PACK_DIR = rospkg.RosPack().get_path("fbot_speech")
RESOURCES_DIR = os.path.join(PACK_DIR, "resources")
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
FILENAME = os.path.join(AUDIO_DIR, "speech_input.wav")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")

DEFAULT_LANGUAGE = 'en'


class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer')

        # Parameters
        self.declare_parameter("noise_adjust_duration", 1)
        self.declare_parameter("whisper_model", "small")
        self.declare_parameter("phrase_time_limit", 8)
        self.declare_parameter("timeout", 5)
        self.declare_parameter("sample_rate", 16000)

        self.noise_adjust_duration = self.get_parameter("noise_adjust_duration").get_parameter_value().integer_value
        self.whisper_model = self.get_parameter("whisper_model").get_parameter_value().string_value
        self.phrase_time_limit = self.get_parameter("phrase_time_limit").get_parameter_value().integer_value
        self.timeout = self.get_parameter("timeout").get_parameter_value().integer_value
        self.sample_rate = self.get_parameter("sample_rate").get_parameter_value().integer_value

        # Recognizer initialization
        self.recognizer = Recognizer()

        # Service
        self.declare_parameter("services/speech_recognizer/service", "/fbot_speech/sr/speech_recognizer")
        recognizer_service_param = self.get_parameter("services/speech_recognizer/service").get_parameter_value().string_value

        self.speech_recognition_service = self.create_service(SpeechToText, recognizer_service_param, self.handle_recognition)

        self.get_logger().info("Speech Recognizer is on!")

    def handle_recognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        with Microphone(sample_rate=self.sample_rate) as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=self.noise_adjust_duration)

        playsound(TALK_AUDIO, block=True)

        with Microphone(sample_rate=self.sample_rate) as source:
            try:
                audio = self.recognizer.listen(source, phrase_time_limit=self.phrase_time_limit, timeout=self.timeout)

                prompt = req.prompt
                lang = req.lang if req.lang != '' else DEFAULT_LANGUAGE

                model = self.whisper_model
                if lang == 'en':
                    if not model.endswith('.en'):
                        model = model + '.en'
                else:
                    if model.endswith('.en'):
                        model = model[:-3]

                self.get_logger().info(f'Prompt to make easier the recognition: {prompt}')
                text = self.recognizer.recognize_whisper(audio, model, language=lang, initial_prompt=prompt,
                                                         load_options={'download_root': RESOURCES_DIR, 'in_memory': True}).lower()

            except WaitTimeoutError:
                text = ''

        response.text = text
        return response


def main(args=None):
    rclpy.init(args=args)

    node = SpeechRecognizerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
