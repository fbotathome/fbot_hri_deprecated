#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.service import Service
from rclpy.publisher import Publisher
from std_srvs.srv import Empty
from audio_common_msgs.msg import AudioData, AudioInfo
from fbot_speech_msgs.srv import AudioPlayerByData, SynthesizeSpeech
from fbot_speech_msgs.msg import SynthesizeSpeechMessage

import os
import torch
import numpy as np
from scipy.io import wavfile

from TTS.tts.configs.xtts_config import XttsConfig
from TTS.tts.models.xtts import Xtts
from TTS.api import TTS

HOME_DIR = os.path.expanduser("~")

PACK_DIR = os.path.join(os.getenv("ROS_PACKAGE_PATH", ""), "fbot_speech")
VOICES_DIR = os.path.join(PACK_DIR, "voices/")
MODELS_DIR = os.path.join(HOME_DIR, ".local/share/tts")

AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
FILENAME = os.path.join(AUDIO_DIR, "talk.wav")

WAV_EXTENSION = ".wav"

class XTTSSpeechSynthesizerNode(Node):
    def __init__(self, node_name="speech_synthesizer"):
        super().__init__(node_name)

        self.__read_params()
        self.sample_rate = 24000

        self.get_logger().info("Loading TTS model: " + self.tts_model_name)
        try:
            TTS(self.tts_model_name)
        except Exception as e:
            self.get_logger().error("Error loading TTS model: " + str(e))
            self.get_logger().error("Use some model from the list: " + str(TTS.list_models()))
            return

        self.tts_model_dir = os.path.join(MODELS_DIR, self.tts_model_name.replace("/", "--"))
        self.get_logger().debug("TTS model directory: " + self.tts_model_dir)

        if not os.path.exists(self.tts_model_dir):
            self.get_logger().error("TTS model directory not found: " + self.tts_model_dir)
            exit()
        
        self.__load_tts_model()
        self.__compute_voice_latents()
        self.__init_comm()

        self.get_logger().info("Speech Synthesizer is on!")

    def __load_tts_model(self):
        config = XttsConfig()
        config.load_json(os.path.join(self.tts_model_dir, self.tts_config_file_name))
        self.tts_model = Xtts.init_from_config(config)
        self.tts_model.load_checkpoint(config, checkpoint_dir=self.tts_model_dir, use_deepspeed=self.tts_use_deepspeed)

        if self.tts_use_cuda and torch.cuda.is_available():
            self.tts_model.cuda()
        else:
            self.get_logger().warn("CUDA is not available, using CPU!")
            self.tts_model.cpu()

    def __read_params(self):
        self.robot_name = self.get_parameter_or("robot_name", "BORIS")
        self.use_streaming = self.get_parameter_or("tts/use_streaming", False)
        self.speech_synthesizer_service_name = self.get_parameter_or("services/speech_synthesizer/service", "/fbot_speech/ss/say_something")
        self.audio_player_by_data_service_name = self.get_parameter_or("services/audio_player_by_data/service", "/fbot_speech/ap/audio_player_by_data")
        self.stream_start_service_name = self.get_parameter_or("services/stream_start/service", "/fbot_speech/ap/stream_start")
        self.stream_stop_service_name = self.get_parameter_or("services/stream_stop/service", "/fbot_speech/ap/stream_stop")
        self.stream_data_topic_name = self.get_parameter_or("subscribers/stream_data/topic", "/fbot_speech/ap/stream_data")
        self.tts_model_name = self.get_parameter_or("tts/model_name", "tts_models/multilingual/multi-dataset/xtts_v2")
        self.tts_config_file_name = self.get_parameter_or("tts/config_file_name", "config.json")
        self.tts_use_deepspeed = self.get_parameter_or("tts/use_deepspeed", False)
        self.tts_use_cuda = self.get_parameter_or("tts/use_cuda", True)
        self.tts_temperature = self.get_parameter_or("tts/temperature", 0.7)
        self.tts_speed = self.get_parameter_or("tts/speed", 1.0)
        self.tts_enable_text_splitting = self.get_parameter_or("tts/enable_text_splitting", True)

    def __init_comm(self):
        self.speech_synthesizer_service = self.create_service(SynthesizeSpeech, self.speech_synthesizer_service_name, self.synthesize_speech)
        self.stream_publisher = self.create_publisher(AudioData, self.stream_data_topic_name, 10)

    def __compute_voice_latents(self):
        robot_voices_dir = os.path.join(VOICES_DIR, self.robot_name)
        if not os.path.exists(robot_voices_dir):
            self.get_logger().error("Robot voices directory not found: " + robot_voices_dir)
            exit()

        robot_voices_files = [os.path.join(robot_voices_dir, filename) for filename in os.listdir(robot_voices_dir) if filename.endswith(WAV_EXTENSION)]
        gpt_cond_latent, speaker_embedding = self.tts_model.get_conditioning_latents(audio_path=robot_voices_files)

        self.voice_latents = {"gpt_cond_latent": gpt_cond_latent, "speaker_embedding": speaker_embedding}

    def synthesize_speech(self, request, response):
        text = request.text
        lang = request.lang if request.lang else "en"

        use_streaming = self.use_streaming

        if request.force_stream_mode:
            use_streaming = True

        self.get_logger().debug("Synthesizing speech: " + text)
        self.get_logger().debug("Language: " + lang)

        if not use_streaming:
            out = self.tts_model.inference(
                text,
                lang,
                self.voice_latents['gpt_cond_latent'],
                self.voice_latents['speaker_embedding'],
                temperature=self.tts_temperature,
                speed=self.tts_speed,
                enable_text_splitting=self.tts_enable_text_splitting,
            )
            wav = torch.tensor(out['wav'])
            wav_data = (wav.view(-1).cpu().numpy() * 32768).astype(np.int16)

            wavfile.write(FILENAME, self.sample_rate, wav_data)

            audio_data = AudioData()
            audio_data.data = wav_data.tobytes()
            audio_info = AudioInfo()
            audio_info.sample_rate = self.sample_rate
            audio_info.channels = 1
            audio_info.sample_format = '8'

            self.get_logger().info(f"Audio data written to {FILENAME}")

            response.success = True
            return response
        else:
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)

    node = XTTSSpeechSynthesizerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
