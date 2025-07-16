#!/usr/bin/env python3
# coding: utf-8
import numpy as np
import riva.client

from fbot_speech_msgs.srv import AudioPlayerByData, SynthesizeSpeech
from fbot_speech_msgs.msg import SynthesizeSpeechMessage
from audio_common_msgs.msg import AudioData, AudioInfo
from fbot_speech_scripts.wav_to_mouth import WavToMouth
import rclpy
from rclpy.node import Node
from threading import Event

class SpeechSynthesizerNode(WavToMouth):
    def __init__(self):
        super().__init__(node_name='speech_synthesizer')
        self.get_logger().info("Initializing Speech Synthesizer Node...")
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        self.event = Event()
        auth = riva.client.Auth(uri=self.riva_url)
        self.riva_tts = riva.client.SpeechSynthesisService(auth)
        self.get_logger().info("Speech Synthesizer Node initialized!")

    def initRosComm(self):
        # Create subscriber
        self.synthesizerSubscriber = self.create_subscription(SynthesizeSpeechMessage, self.synthesizer_subscriber_param, self.synthesizeSpeechCallback, 10)
        # Create service
        self.synthesizerService = self.create_service(SynthesizeSpeech, self.synthesizer_service_param, self.synthesizeSpeech)        

    def declareParameters(self):
        self.declare_parameter('tts_configs.language_code', 'en-US')
        self.declare_parameter('tts_configs.sample_rate_hz', 44100)
        self.declare_parameter('tts_configs.voice_name', 'English-US')
        self.declare_parameter('riva.url', 'localhost:50051')
        self.declare_parameter('services.audio_player_by_data.service', '/fbot_speech/ap/audio_player_by_data')
        self.declare_parameter('services.speech_synthesizer.service', '/fbot_speech/ss/say_something')
        self.declare_parameter('subscribers.speech_synthesizer.topic', '/fbot_speech/ss/say_something')

    def readParameters(self):
        self.audio_player_by_data_service_param = self.get_parameter('services.audio_player_by_data.service').get_parameter_value().string_value
        self.synthesizer_service_param = self.get_parameter('services.speech_synthesizer.service').get_parameter_value().string_value
        self.synthesizer_subscriber_param = self.get_parameter('subscribers.speech_synthesizer.topic').get_parameter_value().string_value
        self.configs = {
            "language_code": self.get_parameter('tts_configs.language_code').get_parameter_value().string_value,
            "sample_rate_hz": self.get_parameter('tts_configs.sample_rate_hz').get_parameter_value().integer_value,
            "voice_name": self.get_parameter('tts_configs.voice_name').get_parameter_value().string_value,
        }
        self.riva_url = self.get_parameter('riva.url').get_parameter_value().string_value

    def synthesizeSpeech(self, request: SynthesizeSpeech.Request, response: SynthesizeSpeech.Response):
        """
        @brief Synthesize speech from text using Riva TTS.
        @param request: The request object containing the text to synthesize.
        @return: The response object indicating success or failure.
        """
        config = self.configs
        speech = request.text
        try:
            # Call Riva TTS to synthesize the speech
            resp = self.riva_tts.synthesize(
                custom_dictionary=config, 
                text=speech,
                voice_name=config["voice_name"], 
                sample_rate_hz=config["sample_rate_hz"],
                language_code=config["language_code"],
                encoding=riva.client.AudioEncoding.LINEAR_PCM,
            )
            
            # Convert the response audio to a NumPy array
            audio_samples = np.frombuffer(resp.audio, dtype=np.int16)
            # Prepare the audio data to send to the audio player
            audio_data = AudioData()
            audio_data.uint8_data = audio_samples.tobytes()
            
            audio_info = AudioInfo()
            audio_info.rate = config["sample_rate_hz"]
            audio_info.channels = 1
            audio_info.format = 16    

            try:
                if self.streaming:
                    response.success = False
                    return response

                data = audio_data.uint8_data
                info = audio_info
                self.setDataAndInfo(data, info)

                while self.playAllData() != True:
                    continue
                response.success = self.playAllData()
                self.get_logger().info(f"AllData: {response}")
            except:
                response.success = False
                self.get_logger().error(f"Error while synthesizing speech voice: {e}")
        
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error while synthesizing speech: {e}")
        
        return response
    
    def synthesizeSpeechCallback(self, msg: SynthesizeSpeechMessage):
        """
        @brief Callback function for the speech synthesizer subscriber.
        This function is called when a new message is received on the subscriber topic. It extracts the text and language from the message and calls the synthesizeSpeech function.
        @param msg: The message object containing the text and language to synthesize.
        """
        request = SynthesizeSpeech.Request()
        request.text = msg.text
        request.lang = msg.lang
        self.synthesizeSpeech(request, SynthesizeSpeech.Response())

def main(args=None):
    rclpy.init(args=args)
    node = SpeechSynthesizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
