#!/usr/bin/env python3
# coding: utf-8
import os
import torch
import numpy as np
import rospkg
import pickle
import warnings

from scipy.io import wavfile
from termcolor import colored
from rclpy.node import Node
from std_srvs.srv import SetBool
from fbot_speech_msgs.srv import AudioPlayer, AudioPlayerByData, SynthesizeSpeech
from fbot_speech_msgs.msg import SynthesizeSpeechMessage
from audio_common_msgs.msg import AudioData, AudioInfo
import rclpy
from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none


MODEL_DIR = ("/home/luisffee/fbot_ws/src/fbot_hri/fbot_speech/include/model/total_count/")
MODEL_NAME = "model.pkl"
MODEL_PATH = os.path.join(MODEL_DIR, MODEL_NAME)
AUDIO_DIR = ("./audios")
FILENAME = str(AUDIO_DIR) + "talk.wav"


class SpeechSynthesizerNode(Node):
    def __init__(self):
        super().__init__('speech_synthesizer')
        self.get_logger().info("Initializing Speech Synthesizer Node...")
        
        # Parameters
        self.tag = self.get_parameter_or("fbot_speech_synthesizer/tag", "kan-bayashi/ljspeech_vits")
        self.vocoder_tag = self.get_parameter_or("fbot_speech_synthesizer/vocoder_tag", "none")
        self.audio_player_by_data_service = self.get_parameter_or("services/audio_player_by_data/service", "/fbot_speech/ap/audio_player_by_data")
        
        self.model = None
        self.load_model()

        self.synthesizer_service = self.create_service(SynthesizeSpeech, '/fbot_speech/ss/say_something', self.synthesizeSpeech)
        self.synthesizer_subscriber = self.create_subscription(SynthesizeSpeechMessage, '/fbot_speech/ss/say_something', self.synthesizeSpeechCallback, 10)

        self.get_logger().info("Speech Synthesizer Node initialized!")

    def load_model(self):
        os.makedirs(MODEL_DIR, exist_ok=True)

        try:
            with open(MODEL_PATH, 'rb') as f:
                self.model = pickle.load(f)
            self.get_logger().info("Model loaded from local storage.")
        except Exception as e:
            self.get_logger().error(f"Failed to load local model: {e}")
            self.model = Text2Speech.from_pretrained(model_tag=str_or_none(self.tag),
                                                     vocoder_tag=str_or_none(self.vocoder_tag),
                                                     device="cpu",
                                                     threshold=0.5,
                                                     minlenratio=0.0,
                                                     maxlenratio=10.0,
                                                     use_att_constraint=False,
                                                     backward_window=1,
                                                     forward_window=3,
                                                     speed_control_alpha=1.15,
                                                     noise_scale=0.333,
                                                     noise_scale_dur=0.333)
            print(self.model)
            self.get_logger().info("Model downloaded from the internet.")
            with open(MODEL_PATH, 'wb') as f:
                pickle.dump(self.model, f)

    def synthesizeSpeech(self, request, response):
        speech = request.text
        lang = request.lang if request.lang else "en"
        
        try:
            with torch.no_grad():
                wav = self.model(speech)["wav"]
                wav_data = (wav.view(-1).cpu().numpy() * 32768).astype(np.int16)
                wavfile.write(FILENAME, self.model.fs, wav_data)
            
            audio_data = AudioData()
            audio_data.uint8_data = wav_data.tobytes()

            audio_info = AudioInfo()
            audio_info.rate = self.model.fs
            audio_info.channels = 1
            audio_info.format = 16

            # Wait for the audio player service
            self.get_logger().info(f"Calling audio player service: {self.audio_player_by_data_service}")
            audio_player_client = self.create_client(AudioPlayerByData, self.audio_player_by_data_service)
            
            while not audio_player_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {self.audio_player_by_data_service} service to become available...")
            
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
            response = SynthesizeSpeech.Response()
            self.synthesizeSpeech(request, response)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechSynthesizerNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
