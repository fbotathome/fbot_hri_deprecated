#!/usr/bin/env python3
# coding: utf-8
import os
import numpy as np
import riva.client
import rospy
import rospkg
import warnings

from scipy.io import wavfile
from termcolor import colored
from std_srvs.srv import SetBool
from fbot_speech_msgs.srv import AudioPlayer, AudioPlayerByData, SynthesizeSpeech
from fbot_speech_msgs.msg import SynthesizeSpeechMessage
from audio_common_msgs.msg import AudioData, AudioInfo
import rclpy
from rclpy.node import Node

# Fetch the warning parameter from ROS parameters
warning = rospy.get_param("warnings", False)
if not warning:
    # Suppress warnings if the parameter is set to False
    warnings.filterwarnings("ignore")

# Get the path of the 'fbot_speech' package
PACK_DIR = rospkg.RosPack().get_path("fbot_speech")
# Construct the path to the audio directory
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
# Set the filename for the generated audio file
FILENAME = str(AUDIO_DIR) + "talk.wav"

class SpeechSynthesizerNode(Node):
    def __init__(self):
        super().__init__('speech_synthesizer')
        
        self.get_logger().info("Initializing Speech Synthesizer Node...")
        
        # Parameters
        self.config_defaults = {
            "language_code": "en-US",
            "sample_rate_hz": 44100,
            "voice_name": "English-US.Male-1",
        }
        
        self.configs = self.get_parameter_or("tts_configs", self.config_defaults)
        
        self.riva_url = self.get_parameter_or("riva/url", "localhost:50051")
        auth = riva.client.Auth(uri=self.riva_url)
        self.riva_tts = riva.client.SpeechSynthesisService(auth)

        # Fetch the audio player by data service parameter
        self.audio_player_by_data_service_param = self.get_parameter_or("services/audio_player_by_data/service", "/fbot_speech/ap/audio_player_by_data")

        # Subscribe to a topic for text input
        self.synthesizer_subscriber_param = self.get_parameter_or("subscribers/speech_synthesizer/topic", "/fbot_speech/ss/say_something")
        self.create_subscription(SynthesizeSpeechMessage, self.synthesizer_subscriber_param, self.synthesizeSpeechCallback, 10)

        # Create the service for speech synthesis
        self.synthesizer_service_param = self.get_parameter_or("services/speech_synthesizer/service", "/fbot_speech/ss/say_something")
        self.create_service(SynthesizeSpeech, self.synthesizer_service_param, self.synthesizeSpeech)

        self.get_logger().info("Speech Synthesizer Node initialized!")

    def synthesizeSpeech(self, request: SynthesizeSpeech.Request, response: SynthesizeSpeech.Response):
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
