#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
from scripts.wav_to_mouth import WavToMouth
from fbot_speech_msgs.srv import (
    AudioPlayer,
    AudioPlayerByData,
    AudioStreamStart
)
from audio_common_msgs.msg import AudioData
from std_srvs.srv import Empty

from termcolor import colored
import warnings

warnings.filterwarnings("ignore")

last_stream_data_timestamp = None
wm = None

class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        # Initialize WavToMouth object
        global wm
        wm = WavToMouth()

        # Get parameters
        self.audio_player_service_param = self.get_parameter_or("services/audio_player/service", "/butia_speech/ap/audio_player")
        self.audio_player_by_data_service_param = self.get_parameter_or("services/audio_player_by_data/service", "/butia_speech/ap/audio_player_by_data")
        self.audio_player_stream_start_service_param = self.get_parameter_or("services/stream_start/service", "/butia_speech/ap/stream_start")
        self.audio_player_stream_stop_service_param = self.get_parameter_or("services/stream_stop/service", "/butia_speech/ap/stream_stop")
        self.audio_player_stream_data_topic_param = self.get_parameter_or("subscribers/stream_data/topic", "/butia_speech/ap/stream_data")
        self.stream_timeout = self.get_parameter_or("stream_timeout", 10)

        # Create services
        self.create_service(AudioPlayer, self.audio_player_service_param, self.to_talk)
        self.create_service(AudioPlayerByData, self.audio_player_by_data_service_param, self.to_talk_by_data)
        self.create_service(AudioStreamStart, self.audio_player_stream_start_service_param, self.audio_stream_start)
        self.create_service(Empty, self.audio_player_stream_stop_service_param, self.stop_stream)

        # Create subscriber
        self.create_subscription(AudioData, self.audio_player_stream_data_topic_param, self.stream_data_callback, 10)

        # Timer to check for stream timeout
        self.create_timer(1.0, self.check_stream_timeout)

        self.get_logger().info(colored("Audio Player is on!", "green"))

    def to_talk(self, request, response):
        if wm.streaming:
            response.success = False
            return response

        filepath = request.audio_path
        wm.set_filepath(filepath)
        wm.play_all_data()

        response.success = True
        return response
        
    def to_talk_by_data(self, request, response):
        if wm.streaming:
            response.success = False
            return response

        data = request.data.data
        info = request.audio_info
        wm.set_data_and_info(data, info)
        wm.play_all_data()

        response.success = True
        return response

    def audio_stream_start(self, request, response):
        global last_stream_data_timestamp
        if wm.streaming:
            response.success = False
            return response
        
        last_stream_data_timestamp = self.get_clock().now()

        info = request.audio_info
        wm.set_audio_info(info)

        wm.start_stream()

        response.success = True
        return response

    def stop_stream(self, request, response):
        global last_stream_data_timestamp
        wm.request_stop_stream()

        last_stream_data_timestamp = None

        while wm.streaming:
            time.sleep(0.1)

        response.success = True
        return response

    def stream_data_callback(self, data):
        global last_stream_data_timestamp
        if wm.streaming:
            last_stream_data_timestamp = self.get_clock().now()
            wm.stream_data_callback(data)

    def check_stream_timeout(self):
        global last_stream_data_timestamp
        if wm.streaming:
            if last_stream_data_timestamp is not None:
                now = self.get_clock().now()
                if now - last_stream_data_timestamp >= rclpy.duration.Duration(seconds=self.stream_timeout):
                    self.get_logger().info('STREAM TIMEOUT')
                    wm.request_stop_stream()
                    last_stream_data_timestamp = None


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    audio_player_node = AudioPlayerNode()
    rclpy.spin(audio_player_node)

    # Clean up
    audio_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
