import rclpy
from rclpy.node import Node
import audioop
import os
import wave
import rospkg
import numpy as np
import struct
import sounddevice as sd
from std_msgs.msg import Int16MultiArray, Int16
from threading import Lock, Event

#fbot_SPEECH_PKG = rospkg.RosPack().get_path("fbot_speech")
#AUDIO = os.path.join(fbot_SPEECH_PKG, "audios/")

def mapRange(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

class WavToMouth(Node):

    def __init__(self):
        super().__init__('wav_to_mouth')  # ROS 2 requires a node name
        
        self.chunk_size = self.get_parameter_or("chunk_size", 2048)


        self.data = []
        self.data_lock = Lock()  # To ensure thread-safe access to self.data

        self.max = 0
        self.min = 0

        self.audio = None
        self.audio_info = None

        self.output = Int16MultiArray()
        self.mouth_gain = self.get_parameter_or("mouth_gain", 2.5)

        self.angle_publisher = self.create_publisher(Int16MultiArray, "mouth", 1)  # ROS 2 publisher
        self.mouth_debug_publisher = self.create_publisher(Int16, "mouth_debug", 1)

        self.streaming = False
        self.request_stream_stop = False

        self.sample_rate = 44100  # Default value; will be updated
        self.channels = 1         # Default value; will be updated

        self.stream = None
        self.stream_lock = Lock()  # To manage access to the stream

        self.playback_done_event = Event()  # Event to signal playback completion

    def divideAudioInChunks(self, audio_data, num_bytes=2):
        chunks = []
        for start in range(0, len(audio_data), self.chunk_size*num_bytes):
            end = min(start + self.chunk_size*num_bytes, len(audio_data))
            chunks.append(audio_data[start:end])
        return chunks

    def _readDataOfAudio(self):
        self.audio = wave.open(self.filepath, "rb")
        audio_data = self.audio.readframes(self.audio.getnframes())
        self.data += self.divideAudioInChunks(audio_data, num_bytes=self.audio.getsampwidth())
        self.sample_rate = self.audio.getframerate()
        self.channels = self.audio.getnchannels()

    def requestStopStream(self):
        self.request_stream_stop = True

    def startStream(self):
        with self.stream_lock:
            if self.streaming:
                self.get_logger().warn("Stream is already running.")  # ROS 2 logger
                return
            self.streaming = True
            self.request_stream_stop = False

            if self.stream is None:
                self.stream = sd.OutputStream(
                    samplerate=self.sample_rate,
                    channels=self.channels,
                    callback=self.audioCallback,
                    blocksize=self.chunk_size,
                    dtype='int16'
                )
                self.stream.start()
                self.get_logger().info("Audio stream started.")
            else:
                self.get_logger().warn("Stream already initialized.")

    def stopStream(self):
        with self.stream_lock:
            if not self.streaming:
                self.get_logger().warn("Stream is not running.")
                return
            self.streaming = False
            self.output.data = [0, 100]
            self.angle_publisher.publish(self.output)
            if self.stream is not None:
                self.stream.stop()
                self.stream.close()
                self.stream = None
                self.get_logger().info("Audio stream stopped.")
            sd.stop()
            self.get_logger().info(f"Streaming: {self.streaming}")
            return not self.streaming

    def setAudioInfo(self, audio_info):
        self.audio_info = audio_info
        self._openStream()

    def setFilepath(self, filepath):
        self.filepath = os.path.join(filepath)
        self._readDataOfAudio()

    def setDataAndInfo(self, data, info):
        self.data += self.divideAudioInChunks(data)
        self.audio_info = info
        self._openStream()

    def _openStream(self):
        if self.audio is not None:
            self.sample_rate = self.audio.getframerate()
            self.channels = self.audio.getnchannels()
        elif self.audio_info is not None:
            self.sample_rate = self.audio_info.rate
            self.channels = self.audio_info.channels

        self.audio = None
        self.audio_info = None

    def _computeChunkRms(self, audio_chunk):
        """Compute the RMS value of an audio chunk."""
        count = len(audio_chunk) // 2
        format = "%dh" % count
        shorts = struct.unpack(format, audio_chunk)
        sum_squares = sum(s ** 2 for s in shorts)
        rms = np.sqrt(sum_squares / count)
        return rms

    def _normalizeRms(self, rms_value, gain=2.5, max_rms=32768):
        """Normalize the RMS value to a range from 0 to 100."""
        rms_value = min(rms_value * gain, max_rms)
        return min(max(int((rms_value / max_rms) * 100), 0), 100)

    def streamDataCallback(self, data):
        with self.data_lock:
            self.data += self.divideAudioInChunks(data)

    def audioCallback(self, outdata, frames, time, status):
        if status:
            self.get_logger().error(f"Stream status: {status}")
        with self.data_lock:
            if len(self.data) == 0:
                # If no data is available, output silence
                outdata[:] = np.zeros((frames, self.channels), dtype='int16')
                # Signal that playback is done
                self.playback_done_event.set()
                return

            # Get the next chunk
            data = self.data.pop(0)

        # Convert bytes to NumPy array
        audio_array = np.frombuffer(data, dtype=np.int16)

        # Reshape based on channels
        if self.channels > 1:
            try:
                audio_array = audio_array.reshape(-1, self.channels)
            except ValueError:
                self.get_logger().error("Audio data size is not compatible with the number of channels.")
                audio_array = np.zeros((frames, self.channels), dtype='int16')

        # Handle cases where the chunk size doesn't match the stream's blocksize
        if len(audio_array) < frames * self.channels:
            # Pad with zeros if the chunk is smaller
            pad_width = (frames * self.channels) - len(audio_array)
            audio_array = np.pad(audio_array, (0, pad_width), 'constant', constant_values=0)
        elif len(audio_array) > frames * self.channels:
            # Trim the chunk if it's larger
            audio_array = audio_array[:frames * self.channels]

        outdata[:] = audio_array.reshape((frames, self.channels))

        # Compute RMS and publish mouth angles
        rms = self._computeChunkRms(data)
        mouth_angle = self._normalizeRms(rms, gain=self.mouth_gain)
        self.output.data = [mouth_angle, abs(100 - mouth_angle)]
        self.angle_publisher.publish(self.output)
        mouth_angle_int16 = Int16()
        mouth_angle_int16.data = mouth_angle
        self.mouth_debug_publisher.publish(mouth_angle_int16)

    def playAllData(self):
        # Clear the playback done event
        self.playback_done_event.clear()
        # Start the audio stream
        self.startStream()
        # Wait until the playback is done
        self.playback_done_event.wait()
        # Stop the stream gracefully
        return self.stopStream()