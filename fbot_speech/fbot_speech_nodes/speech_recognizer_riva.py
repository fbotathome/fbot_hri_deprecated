#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import riva.client
from fbot_speech_msgs.srv import SpeechToTextASR
import riva.client.proto.riva_asr_pb2 as rasr
import riva.client.audio_io
import os
import time
from playsound import playsound
import threading

DEFAULT_LANGUAGE = 'en'
PACK_DIR = os.path.join(os.path.expanduser("~"), 'jetson_ws', 'src', 'fbot_hri', 'fbot_speech')
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
            start_history= 50,
            start_threshold= -1,
            stop_history= 500,
            stop_history_eou= 700,
            stop_threshold= -1.0,
            stop_threshold_eou= -1.0,
        )

        default_device_info = riva.client.audio_io.get_default_input_device_info()
        self.device = default_device_info['index']


    def initRosComm(self):
        self.speech_recognition_service = self.create_service(SpeechToTextASR, self.recognizer_service_param, self.handleRecognition)
        self.audio_player_beep_service = self.create_client(Empty, self.audio_player_beep_param_service)

    def declareParameters(self):
        self.declare_parameter('riva.url', 'localhost:50051')
        self.declare_parameter('stt_mic_timeout', 10)
        self.declare_parameter('services.audio_player_beep.service', '/fbot_speech/ap/audio_beep')
        self.declare_parameter('services.asr_recognizer.service', '/fbot_speech/sr/speech_recognizer_asr')


    def readParameters(self):
        self.audio_player_beep_param_service = self.get_parameter('services.audio_player_beep.service').get_parameter_value().string_value
        self.recognizer_service_param = self.get_parameter('services.speech_recognizer.service').get_parameter_value().string_value
        self.stt_mic_timeout = self.get_parameter('stt_mic_timeout').get_parameter_value().integer_value
        self.riva_url = self.get_parameter('riva.url').get_parameter_value().string_value


    def delayStarterRecorder(self):
        time.sleep(0.75)
        # self.audio_player_beep_service.call(Empty.Request())
        playsound(TALK_AUDIO)
    
    def handleRecognition(self, req: SpeechToTextASR.Request, res: SpeechToTextASR.Response):
        
        with riva.client.audio_io.MicrophoneStream(
                                                        rate =16000,
                                                        chunk=512,
                                                        device=self.device,
                                                    ) as audio_chunk_iterator:
            speech_context = rasr.SpeechContext()
            self.get_logger().error('Passou')
            good_output = ''
            bad_output = ''
            very_bad_output = ''
            delay_starter = threading.Thread(target=self.delayStarterRecorder)

            if req.boosted_lm_words != '':
                speech_context.phrases.extend(req.boosted_lm_words)
                speech_context.boost = req.boost
                self.config.config.speech_contexts.clear()
                self.config.config.speech_contexts.extend([speech_context])
            
            if req.word:
                self.word = True
                self.sentence = False
            else:
                self.word = False
                self.sentence = True


            output = self.riva_asr.streaming_response_generator(
                    audio_chunks=audio_chunk_iterator,
                    streaming_config=self.config)
            
            start = time.time() + self.stt_mic_timeout
            delay_starter.start()
            for response in output:
                if (start > time.time()):
                    if not response.results:
                        continue
                    for result in response.results:
                        if not result.alternatives:
                            continue
                    if result.is_final:
                        self.get_logger().error(f'Alternative: {result.alternatives}')
                        if self.word:
                            if result.alternatives[0].words[0].word in self.boosted_lm_words:
                                if result.alternatives[0].words[0].confidence >=0.6:
                                    good_output = result.alternatives[0].words[0].word
                                    res.text = good_output
                                    audio_chunk_iterator.close()
                                    return res
                                else:
                                    bad_output = result.alternatives[0].words[0].word
                            else:
                                very_bad_output = result.alternatives[0].words[0].word
                        elif self.sentence:
                            for alternative in result.alternatives:
                                for word in alternative.words:
                                    if word.word in self.boosted_lm_words:
                                        res.text = result.alternatives[0].transcript 
                                        audio_chunk_iterator.close()
                                        return res
                                    else:
                                        bad_output = result.alternatives[0].transcript
                else:
                    audio_chunk_iterator.close()
                    if bad_output != '':
                        res.text = bad_output
                        return res
                    else:
                        res.text = very_bad_output
                        return res

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