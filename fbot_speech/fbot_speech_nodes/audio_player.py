#!/usr/bin/env python3

import time
import rclpy
import os
import warnings
from rclpy.node import Node
from fbot_speech_scripts.wav_to_mouth import WavToMouth
from fbot_speech_msgs.srv import AudioPlayer, AudioPlayerByData, AudioStreamStart
from audio_common_msgs.msg import AudioData
from std_srvs.srv import Empty
from playsound import playsound
from ament_index_python.packages import get_package_share_directory
from termcolor import colored

warnings.filterwarnings("ignore")

class AudioPlayerNode(WavToMouth):
    """
    @details This class provides services to play audio files, stream audio data, and stop streaming.
    """
    
    def __init__(self):
        """
        @brief Constructor for the AudioPlayerNode class.
        @details Initializes the node, declares parameters, and sets up services and subscribers.
        """
        super().__init__(node_name='audio_player')
        # Initialize WavToMouth object
        self.last_stream_data_timestamp = None
        ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_speech'), '../../../..'))
        self.file_path = os.path.join(ws_dir, "src", "fbot_hri", "fbot_speech",  "audios", "beep.wav")
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        # Timer to check for stream timeout
        self.create_timer(1.0, self.audioCheckStreamTimeout)
        self.get_logger().info(colored("Audio Player is on!", "green"))

    def initRosComm(self):
        # Create service
        self.audioSpeechFromFileService = self.create_service(AudioPlayer, self.audio_player_service_param, self.audioSpeechFromFile)
        self.audioSpeechFromDataService = self.create_service(AudioPlayerByData, self.audio_player_by_data_service_param, self.audioSpeechFromData)
        self.audioStreamStartService = self.create_service(AudioStreamStart, self.audio_player_stream_start_service_param, self.audioStreamStart)
        self.audioStreamStopService = self.create_service(Empty, self.audio_player_stream_stop_service_param, self.audioStreamStop)
        self.audioBeepService = self.create_service(Empty, self.audio_beep_param, self.audioBeep)

        # Create subscriber
        self.audioPlayerTopicSub = self.create_subscription(AudioData, self.audio_player_stream_data_topic_param, self.audioStreamDataCallback, 10)
        
    def declareParameters(self):
        # Declare parameters
        self.declare_parameter("services.audio_player.service", "/fbot_speech/ap/audio_player")
        self.declare_parameter("services.audio_player_by_data.service", "/fbot_speech/ap/audio_player_by_data")
        self.declare_parameter("services.stream_start.service", "/fbot_speech/ap/stream_start")
        self.declare_parameter("services.stream_stop.service", "/fbot_speech/ap/stream_stop")
        self.declare_parameter("subscribers.stream_data.topic", "/fbot_speech/ap/stream_data")
        self.declare_parameter("services.audioBeep.service", "/fbot_speech/ap/audio_beep")
        self.declare_parameter("stream_timeout", 10)

    def readParameters(self):
        # Get parameters
        self.audio_player_service_param = self.get_parameter("services.audio_player.service").get_parameter_value().string_value
        self.audio_player_by_data_service_param = self.get_parameter("services.audio_player_by_data.service").get_parameter_value().string_value
        self.audio_player_stream_start_service_param = self.get_parameter("services.stream_start.service").get_parameter_value().string_value
        self.audio_player_stream_stop_service_param = self.get_parameter("services.stream_stop.service").get_parameter_value().string_value
        self.audio_player_stream_data_topic_param = self.get_parameter("subscribers.stream_data.topic").get_parameter_value().string_value
        self.audio_beep_param = self.get_parameter("services.audioBeep.service").get_parameter_value().string_value
        self.stream_timeout = self.get_parameter("stream_timeout").get_parameter_value().integer_value

    def audioSpeechFromFile(self, request, response):
        """
        @brief Service callback to play audio from a file.
        This function is called when the service is requested. It sets the file path for the audio player and plays the audio.
        @param request: Request object containing the audio file path.
        @return: Response object indicating success or failure.
        """
        if self.streaming:
            response.success = False
            return response

        filepath = request.audio_path
        self.setFilepath(filepath)
        self.playAllData()

        response.success = True
        return response
    
    def audioBeep(self, request, response):
        playsound(self.file_path)
        
        return response
        
    def audioSpeechFromData(self, request: AudioPlayerByData.Request, response: AudioPlayerByData.Response):
        """
        @brief Service callback to play audio from raw data.
        This function is called when the service is requested. It sets the audio data and info for the audio player and plays the audio.
        @param request: Request object containing the audio data and info.
        @return: Response object indicating success or failure.
        """
        if self.streaming:
            response.success = False
            return response

        data = request.data.uint8_data
        info = request.audio_info
        self.setDataAndInfo(data, info)
        self.playAllData()

        response.success = True
        return response

    def audioStreamStart(self, request, response):
        """
        @brief Service callback to start audio streaming.
        This function is called when the service is requested. It sets the audio info for the audio player and starts streaming.
        @param request: Request object containing the audio info.
        @return: Response object indicating success or failure.
        """
        if self.streaming:
            response.success = False
            return response
        
        self.last_stream_data_timestamp = self.get_clock().now()
        info = request.audio_info
        self.setAudioInfo(info)
        self.startStream()

        response.success = True
        return response

    def audioStreamStop(self, request, response):
        """
        @brief Service callback to stop audio streaming.
        This function is called when the service is requested. It stops the audio streaming and resets the last stream data timestamp.
        @param request: Request object (not used).
        @return: Response object indicating success or failure.
        """
        self.requestStopStream()
        self.last_stream_data_timestamp = None
        while self.streaming:
            time.sleep(0.1)
        response.success = True
        return response

    def audioStreamDataCallback(self, data: AudioData = None):
        """
        @brief Callback function for audio stream data.
        This function is called when audio stream data is received. It updates the last stream data timestamp and processes the data.
        @param data: Audio data received from the stream.
        @return: None
        """
        if self.streaming:
            self.last_stream_data_timestamp = self.get_clock().now()
            self.streamDataCallback(data)

    def audioCheckStreamTimeout(self):
        """
        @brief Check for stream timeout.
        This function checks if the stream has timed out based on the last stream data timestamp. If it has, it stops the stream.
        @return: None
        """
        if self.streaming:
            if self.last_stream_data_timestamp is not None:
                now = self.get_clock().now()
                if now - self.last_stream_data_timestamp >= rclpy.duration.Duration(seconds=self.stream_timeout):
                    self.get_logger().info('STREAM TIMEOUT')
                    self.requestStopStream()
                    self.last_stream_data_timestamp = None

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
