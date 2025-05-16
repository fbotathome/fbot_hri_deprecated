#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import SpeechToText
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
        self.noise_adjust_duration = self.get_parameter_or("noise_adjust_duration", 1)
        self.whisper_model = self.get_parameter_or("whisper_model", "small")
        self.phrase_time_limit = self.get_parameter_or("phrase_time_limit", 8)
        self.timeout = self.get_parameter_or("timeout", 5)
        self.sample_rate = self.get_parameter_or("sample_rate", 16000)

        # Recognizer initialization
        self.recognizer = Recognizer()

        # Service
        recognizer_service_param = self.get_parameter_or("services/speech_recognizer/service", "/fbot_speech/sr/speech_recognizer")

        self.speech_recognition_service = self.create_service(SpeechToText, recognizer_service_param, self.handleRecognition)

        self.get_logger().info("Speech Recognizer is on!")

    def handleRecognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
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
