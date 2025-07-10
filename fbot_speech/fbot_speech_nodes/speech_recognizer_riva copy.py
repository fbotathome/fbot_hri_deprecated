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
                model=args.model_name,
                max_alternatives=1,
                profanity_filter=False,
                enable_automatic_punctuation=False,
                verbatim_transcripts=True,
                sample_rate_hertz=args.sample_rate_hz,
                audio_channel_count=1,
                enable_word_time_offsets = True,

            ),
            interim_results=True
        )


        speech_context = rasr.SpeechContext()
        speech_context.phrases.extend(boosted_lm_words)
        speech_context.boost = boosted_lm_score
        self.config.speech_contexts.append(speech_context)


        endpointing_config = rasr.EndpointingConfig()
        if start_history > 0:
            endpointing_config.start_history = start_history
        if start_threshold > 0:
            endpointing_config.start_threshold = start_threshold
        if stop_history > 0:
            endpointing_config.stop_history = stop_history
        if stop_history_eou > 0:
            endpointing_config.stop_history_eou = stop_history_eou
        if stop_threshold > 0:
            endpointing_config.stop_threshold = stop_threshold
        if stop_threshold_eou > 0:
            endpointing_config.stop_threshold_eou = stop_threshold_eou
        self.config.endpointing_config.CopyFrom(endpointing_config)

    def delayStarterRecorder(self):
        

    def handleRecognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        config = self.configs
        speech = request.text
        try:
             with riva.client.audio_io.MicrophoneStream(
                args.sample_rate_hz,
                args.file_streaming_chunk,
                device=args.input_device,
            ) as audio_chunk_iterator:
                riva.client.print_streaming(
                    responses=self.riva_asr.streaming_response_generator(
                        audio_chunks=audio_chunk_iterator,
                        streaming_config=config,
                    )
                )

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