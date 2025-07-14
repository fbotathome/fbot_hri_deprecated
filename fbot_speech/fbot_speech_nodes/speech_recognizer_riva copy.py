#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import numpy as np
import riva.client
from fbot_speech_msgs.srv import SpeechToText
import riva.client.proto.riva_asr_pb2 as rasr
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
        self.get_logger().info("Initializing Speech Recognizer Node...")
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        auth = riva.client.Auth(uri=self.riva_url)
        self.riva_asr = riva.client.ASRService(auth)
        self.config = riva.client.StreamingRecognitionConfig(
                                                            config = riva.client.RecognitionConfig(
                                                                    encoding=riva.client.AudioEncoding.LINEAR_PCM,
                                                                    language_code='en-US',
                                                                    max_alternatives=1,
                                                                    profanity_filter=False,
                                                                    enable_automatic_punctuation=False,
                                                                    verbatim_transcripts=False,
                                                                    sample_rate_hertz=16000,
                                                                    audio_channel_count=1,
                                                                ),
                                                            interim_results=False)

        riva.client.add_endpoint_parameters_to_config(
            self.config,
            start_history= 100,
            start_threshold= -1,
            stop_history= 500,
            stop_history_eou= 700,
            stop_threshold= -1.0,
            stop_threshold_eou= -1.0,
        )

        default_device_info = riva.client.audio_io.get_default_input_device_info()
        self.device = default_device_info['index']

        self.word = True
        self.sentence = not self.word

        

    def handleRecognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        with riva.client.audio_io.MicrophoneStream(
                                                        rate =16000,
                                                        chunk=512,
                                                        device=self.device,
                                                    ) as audio_chunk_iterator:
            speech_context = rasr.SpeechContext()

            resultado_muito_ruim = ''
            resultado_bom = ''
            resultado_ruim = ''

            self.restaurant = ['coke', 'pizza', 'wine', 'water', 'sandwich']

            self.boosted_lm_words = ['James', 'Michael', 'Robert', 'John', 'David', 'William', 'Richard', 'Joseph', 'Thomas',
                            'Christopher', 'Mary',  'Patricia', 'Jennifer', 'Linda', 'Elizabeth', 'Barbara', 'Susan', 'Jessica', 'Karen', 'Sarah'] + self.restaurant

            words = ['Vitor', 'Andrei', 'Richard', 'PEPSI', 'Rice'] + self.boosted_lm_words
            boosted = 100


            speech_context.phrases.extend(words)
            speech_context.boost = boosted
            self.config.config.speech_contexts.clear()
            self.config.config.speech_contexts.extend([speech_context])
            
            bad_output = ''
            very_bad_output = ''


            output = self.asr_service.streaming_response_generator(
                    audio_chunks=audio_chunk_iterator,
                    streaming_config=self.config)
            
            start = time.time() + self.timeout
            for response in output:
                if (start > time.time()):
                    if self.word:
                        if not response.results:
                            continue
                        for result in response.results:
                            if not result.alternatives:
                                continue
                        if result.is_final:
                            if result.alternatives[0].words[0].word in words:
                                if result.alternatives[0].words[0].confidence >=0.6:
                                    resultado_bom = result.alternatives[0].words[0].word
                                    return resultado_bom
                                else:
                                    resultado_ruim = result.alternatives[0].words[0].word
                            else:
                                resultado_muito_ruim = result.alternatives[0].words[0].word
                    elif self.sentence:
                        if not response.results:
                            continue
                        for result in response.results:
                            if not result.alternatives:
                                continue
                        if result.is_final:
                            for alternative in result.alternatives:
                                for word in alternative.words:
                                    if word.word in words:
                                        return result.alternatives[0].transcript 
                                    else:
                                        resultado_ruim = result.alternatives[0].transcript
                else:
                    if resultado_ruim != '':
                        return resultado_ruim
                    else:
                        return resultado_muito_ruim


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