#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from fbot_speech_scripts.wav_to_mouth import WavToMouth
from fbot_speech_msgs.srv import AudioPlayer, AudioPlayerByData, AudioStreamStart
from audio_common_msgs.msg import AudioData
from std_srvs.srv import Empty

from termcolor import colored
import warnings

warnings.filterwarnings("ignore")

last_stream_data_timestamp = None
wm = None

class AudioPlayerNode(Node):
    """
    @brief Class to handle audio playback and streaming.
    @details This class provides services to play audio files, stream audio data, and stop streaming.

    """
    
    def __init__(self):
        """
        @brief Constructor for the AudioPlayerNode class.
        @details Initializes the node, declares parameters, and sets up services and subscribers.
        """
        super().__init__('audio_player')
        # Initialize WavToMouth object
        global wm
        wm = WavToMouth()
        self.declareParameters()
        self.readParameters()
        self.init_rosComm()
        # Timer to check for stream timeout
        self.create_timer(1.0, self.checkStreamTimeout)
        self.get_logger().info(colored("Audio Player is on!", "green"))

    def init_rosComm(self):
        # Create service
        self.audioPlayerService = self.create_service(AudioPlayer, self.audio_player_service_param, self.toTalk)
        self.audioByDataService = self.create_service(AudioPlayerByData, self.audio_player_by_data_service_param, self.toTalkByData)
        self.audioStreamStartService = self.create_service(AudioStreamStart, self.audio_player_stream_start_service_param, self.audioStreamStart)
        self.audioStreamStopService = self.create_service(Empty, self.audio_player_stream_stop_service_param, self.stopStream)
        # Create subscriber
        self.audioPlayerTopicSub = self.create_subscription(AudioData, self.audio_player_stream_data_topic_param, self.streamDataCallback, 10)
        
    def declareParameters(self):
        # Declare parameters
        self.declare_parameter("services.audio_player.service", "/fbot_speech/ap/audio_player")
        self.declare_parameter("services.audio_player_by_data.service", "/fbot_speech/ap/audio_player_by_data")
        self.declare_parameter("services.stream_start.service", "/fbot_speech/ap/stream_start")
        self.declare_parameter("services.stream_stop.service", "/fbot_speech/ap/stream_stop")
        self.declare_parameter("subscribers.stream_data.topic", "/fbot_speech/ap/stream_data")
        self.declare_parameter("stream_timeout", 10)

    def readParameters(self):
        # Get parameters
        self.audio_player_service_param = self.get_parameter("services.audio_player.service").get_parameter_value().string_value
        self.audio_player_by_data_service_param = self.get_parameter("services.audio_player_by_data.service").get_parameter_value().string_value
        self.audio_player_stream_start_service_param = self.get_parameter("services.stream_start.service").get_parameter_value().string_value
        self.audio_player_stream_stop_service_param = self.get_parameter("services.stream_stop.service").get_parameter_value().string_value
        self.audio_player_stream_data_topic_param = self.get_parameter("subscribers.stream_data.topic").get_parameter_value().string_value
        self.stream_timeout = self.get_parameter("stream_timeout").get_parameter_value().integer_value

    def toTalk(self, request, response):
        """
        @brief Service callback to play audio from a file.
        This function is called when the service is requested. It sets the file path for the audio player and plays the audio.
        @param request: Request object containing the audio file path.
        @return: Response object indicating success or failure.
        """
        if wm.streaming:
            response.success = False
            return response

        filepath = request.audio_path
        wm.setFilepath(filepath)
        wm.playAllData()

        response.success = True
        return response
        
    def toTalkByData(self, request, response):
        """
        @brief Service callback to play audio from raw data.
        This function is called when the service is requested. It sets the audio data and info for the audio player and plays the audio.
        @param request: Request object containing the audio data and info.
        @return: Response object indicating success or failure.
        """
        if wm.streaming:
            response.success = False
            return response

        data = request.data.uint8_data
        info = request.audio_info
        wm.setDataAndInfo(data, info)
        wm.playAllData()

        response.success = True
        return response

    def audioStreamStart(self, request, response):
        """
        @brief Service callback to start audio streaming.
        This function is called when the service is requested. It sets the audio info for the audio player and starts streaming.
        @param request: Request object containing the audio info.
        @return: Response object indicating success or failure.
        """
        global last_stream_data_timestamp
        if wm.streaming:
            response.success = False
            return response
        
        last_stream_data_timestamp = self.get_clock().now()
        info = request.audio_info
        wm.setAudioInfo(info)
        wm.startStream()

        response.success = True
        return response

    def stopStream(self, request, response):
        """
        @brief Service callback to stop audio streaming.
        This function is called when the service is requested. It stops the audio streaming and resets the last stream data timestamp.
        @param request: Request object (not used).
        @return: Response object indicating success or failure.
        """
        global last_stream_data_timestamp
        wm.requestStopStream()
        last_stream_data_timestamp = None
        while wm.streaming:
            time.sleep(0.1)
        response.success = True
        return response

    def streamDataCallback(self, data):
        """
        @brief Callback function for audio stream data.
        This function is called when audio stream data is received. It updates the last stream data timestamp and processes the data.
        @param data: Audio data received from the stream.
        @return: None
        """
        global last_stream_data_timestamp
        if wm.streaming:
            last_stream_data_timestamp = self.get_clock().now()
            wm.streamDataCallback(data)

    def checkStreamTimeout(self):
        """
        @brief Check for stream timeout.
        This function checks if the stream has timed out based on the last stream data timestamp. If it has, it stops the stream.
        @return: None
        """
        global last_stream_data_timestamp
        if wm.streaming:
            if last_stream_data_timestamp is not None:
                now = self.get_clock().now()
                if now - last_stream_data_timestamp >= rclpy.duration.Duration(seconds=self.stream_timeout):
                    self.get_logger().info('STREAM TIMEOUT')
                    wm.requestStopStream()
                    last_stream_data_timestamp = None

def main(args=None):
    rclpy.init(args=args)
    # Create and spin the node
    audio_player_node = AudioPlayerNode()
    rclpy.spin(audio_player_node)
    # Clean up
    audio_player_node.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()
