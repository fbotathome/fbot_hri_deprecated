#!/usr/bin/env python3
# coding: utf-8
import os
import numpy as np
import rospkg
import warnings
import requests

from scipy.io import wavfile
from termcolor import colored
from std_srvs.srv import SetBool
from fbot_speech_msgs.srv import AudioPlayer, AudioPlayerByData, SynthesizeSpeech
from fbot_speech_msgs.msg import SynthesizeSpeechMessage
from audio_common_msgs.msg import AudioData, AudioInfo
import rclpy
from rclpy.node import Node

# Get the path of the 'fbot_speech' package
#PACK_DIR = rospkg.RosPack().get_path("fbot_speech")
# Construct the path to the audio directory
#AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
# Set the filename for the generated audio file
#FILENAME = str(AUDIO_DIR) + "talk.wav"

class SpeechSynthesizerNode(Node):
    def __init__(self):
        super().__init__('speech_synthesizer')
        
        self.get_logger().info("Initializing Speech Synthesizer Node...")
        
        # Parameters
        self.config_defaults = {
            "sample_rate_hz": 22050,
        }
        
        self.configs = self.get_parameter_or("tts_configs", self.config_defaults)
        

        # Fetch the audio player by data service parameter
        self.audio_player_by_data_service_param = self.get_parameter_or("services/audio_player_by_data/service", "/fbot_speech/ap/audio_player_by_data")

        # Subscribe to a topic for text input
        self.synthesizer_subscriber_param = self.get_parameter_or("subscribers/speech_synthesizer/topic", "/fbot_speech/ss/say_something")
        self.create_subscription(SynthesizeSpeechMessage, self.synthesizer_subscriber_param, self.synthesizeSpeechCallback, 10)

        # Create the service for speech synthesis
        self.synthesizer_service_param = self.get_parameter_or("services/speech_synthesizer/service", "/fbot_speech/ss/say_something")
        self.create_service(SynthesizeSpeech, self.synthesizer_service_param, self.synthesizeSpeech)

        self.get_logger().info("Speech Synthesizer Node initialized!")
    
    def fetch_synthesized_speech(self, text, port=5001):
        url = f'http://localhost:{port}'
        params = {'text': text}
        response = requests.get(url, params=params, stream=True)
        
        if response.status_code == 200:
            audio_data = b""
            for chunk in response.iter_content(chunk_size=8192):
                audio_data += chunk
            return audio_data
        else:
            rospy.logerr(f"Failed to fetch synthesized speech: {response.status_code}")
            return None
        

    def synthesizeSpeech(self, request: SynthesizeSpeech.Request, response: SynthesizeSpeech.Response):
        config = self.configs
        speech = request.text
        
        try:
            if lang == "pt": # Portuguese
                # Fetch the synthesized speech from the external service
                audio_data_bytes = self.fetch_synthesized_speech(speech, port=5002)
                if audio_data_bytes is None:
                    response = SynthesizeSpeechResponse()
                    response.success = False
                    return response
                audio_samples = np.frombuffer(audio_data_bytes, dtype=np.int16)
            else: # English
                # Fetch the synthesized speech from the external service
                audio_data_bytes = self.fetch_synthesized_speech(speech, port=5001)
                if audio_data_bytes is None:
                    response = SynthesizeSpeechResponse()
                    response.success = False
                    return response
                audio_samples = np.frombuffer(audio_data_bytes, dtype=np.int16)

            # Prepare the audio data to send to the audio player
            audio_data = AudioData()
            audio_data.uint8_data = audio_samples.tobytes()
            
            audio_info = AudioInfo()
            audio_info.rate = config["sample_rate_hz"]
            audio_info.channels = 1
            audio_info.format = 16

            # Wait for the audio player service
            self.get_logger().info(f"Calling audio player service: {self.audio_player_by_data_service_param}")
            audio_player_client = self.create_client(AudioPlayerByData, self.audio_player_by_data_service_param)
            
            while not audio_player_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {self.audio_player_by_data_service_param} service...")
            
            # Create the request and send it
            audio_player_request = AudioPlayerByData.Request()
            audio_player_request.data = audio_data
            audio_player_request.audio_info = audio_info
            future = audio_player_client.call_async(audio_player_request)
            future.result()

            response.success = True
            self.get_logger().info(f"Audio data played successfully.")
        
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error while synthesizing speech: {e}")
        
        return response

    def synthesizeSpeechCallback(self, msg: SynthesizeSpeechMessage):
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
