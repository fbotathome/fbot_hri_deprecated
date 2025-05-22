#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import threading
import time
import rclpy

from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import SpeechToText
from playsound import playsound
from ament_index_python.packages import get_package_share_directory
from RealtimeSTT import AudioToTextRecorder

DEFAULT_LANGUAGE = 'en'

class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer')
        self.get_logger().info("Initializing Speech Recognizer Node...")
        self.declareParameters()
        self.readParameters()
        self.initRosComm()

        ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_behavior'), '../../../..'))
        self.talk_audio = os.path.join(ws_dir, "src", "fbot_hri", "fbot_speech", "audios", "beep.wav")


        # Global recorder variable
        self.recorder =  AudioToTextRecorder(**self.configs)

        self.get_logger().info("Speech Recognizer is on!")

    def initRosComm(self):
        # Create service
        self.speech_recognition_service = self.create_service(SpeechToText, self.recognizer_service_param, self.handleRecognition)
    
    def declareParameters(self):
        self.declare_parameter('stt_configs.compute_type', "float32")
        self.declare_parameter('stt_configs.spinner', False)
        self.declare_parameter('stt_configs.model', 'distil-small.en')
        self.declare_parameter('stt_configs.silero_sensitivity', 0.6)
        self.declare_parameter('stt_configs.device', 'cpu')
        self.declare_parameter('stt_configs.webrtc_sensitivity', 1)
        self.declare_parameter('stt_configs.post_speech_silence_duration', 1.2)
        self.declare_parameter('stt_configs.min_length_of_recording', 3.0)
        self.declare_parameter('stt_configs.min_gap_between_recordings', 0.0)
        self.declare_parameter('stt_configs.enable_realtime_transcription', False)
        self.declare_parameter('stt_configs.silero_deactivity_detection', True)
        self.declare_parameter('stt_configs.initial_prompt', 'Boris')
        self.declare_parameter('stt_mic_timeout', 10)
        self.declare_parameter('services.speech_recognizer.service', '/fbot_speech/sr/speech_recognizer')

    def readParameters(self):
        self.stt_mic_timeout = self.get_parameter('stt_mic_timeout').get_parameter_value().integer_value
        self.configs = {
            "spinner": self.get_parameter('stt_configs.spinner').get_parameter_value().bool_value,
            "model": self.get_parameter('stt_configs.model').get_parameter_value().string_value,
            "silero_sensitivity":  self.get_parameter('stt_configs.silero_sensitivity').get_parameter_value().double_value,
            "device": self.get_parameter('stt_configs.device').get_parameter_value().string_value,
            "webrtc_sensitivity": self.get_parameter('stt_configs.webrtc_sensitivity').get_parameter_value().integer_value,
            "post_speech_silence_duration": self.get_parameter('stt_configs.post_speech_silence_duration').get_parameter_value().double_value,
            "min_length_of_recording": self.get_parameter('stt_configs.min_length_of_recording').get_parameter_value().double_value,
            "min_gap_between_recordings": self.get_parameter('stt_configs.min_gap_between_recordings').get_parameter_value().double_value,
            "enable_realtime_transcription": self.get_parameter('stt_configs.enable_realtime_transcription').get_parameter_value().bool_value,
            "silero_deactivity_detection": self.get_parameter('stt_configs.silero_deactivity_detection').get_parameter_value().bool_value,
            "initial_prompt": self.get_parameter('stt_configs.initial_prompt').get_parameter_value().string_value,
        }
        self.recognizer_service_param = self.get_parameter('services.speech_recognizer.service').get_parameter_value().string_value


    def delayStarterRecorder(self):
        self.get_logger().info("Beep! Beep! Beep!")
        time.sleep(0.5)
        playsound(self.talk_audio)

    # try:
    #     # Initialize the audio-to-text recorder with the configurations
    #     with AudioToTextRecorder(**configs) as recorder:
    #         # Start the thread to check VAD time
    #         vad_start_time.__setitem__(0, time.time())
    #         vad_thread = threading.Thread(target=check_vad_time, args=(recorder,))
    #         vad_thread.start()
            
    #         # Get the recognized text
    #         text = recorder.text()

    #     try:
    #         # Shutdown the recorder
    #         AudioToTextRecorder.shutdown()
    #     except:
    #         pass
    # except Exception as e:
    #     # Print any exceptions that occur
    #     print(e)
    #     text = ''
    # return SpeechToTextResponse(
    #     text=text
    # )

    

    def handleRecognition(self, req: SpeechToText.Request, response: SpeechToText.Response):
        self.get_logger().info("Handling recognition request...")
         # If a prompt is provided, update the configurations
        if req.prompt != '':
            self.get_logger().info(f'Prompt to make easier the recognition: {req.prompt}')
            self.configs.update({'initial_prompt': req.prompt})

        # Variable to store the start time of VAD detection
        vad_start_time = [None]

        def check_vad_time(recorder):
            while True:
                self.get_logger().info(f"Checking VAD time...{ vad_start_time[0]}")
                seconds_pass = (time.time() - vad_start_time[0])
                if vad_start_time[0] is not None and seconds_pass > self.stt_mic_timeout:
                    self.get_logger().info(f"Stopping listening, too long... {seconds_pass:.1f}s")
                    recorder.stop()
                    recorder.abort()
                    break
                time.sleep(0.1)
        
        self.configs.update({
                            'language': req.lang if req.lang != '' else DEFAULT_LANGUAGE,  # Set the language for recognition
                            'on_recording_start': lambda: self.get_logger().info("Starting Record..."),  # Log message when recording starts
                            'on_vad_detect_start': self.delayStarterRecorder,  # Play beep sound and store start time when voice activity is detected
                            'on_vad_detect_stop': lambda: self.get_logger().info("Finished Listening..."),  # Log message when voice activity stops
                            'on_recording_stop': lambda: self.get_logger().info("Processing...")  # Log message when recording stops
                            })
        
        # Start VAD checking thread
        try:
            # Initialize the audio-to-text recorder with the configurations
            with AudioToTextRecorder(**self.configs) as recorder:
                # Start the thread to check VAD time
                vad_start_time.__setitem__(0, time.time())
                vad_thread = threading.Thread(target=check_vad_time, args=(recorder,))
                vad_thread.start()                
                # Get the recognized text
                text = recorder.text()

            try:
                AudioToTextRecorder.shutdown()
            except Exception as e:
                pass

        except Exception as e:
            # Print any exceptions that occur
            print(e)
            text = ''

            # Return the recognized text
        response.text = text
        return response


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