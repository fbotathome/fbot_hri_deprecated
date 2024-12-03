# CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(fbot_speech_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(audio_common_msgs REQUIRED)

# Custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/SynthesizeSpeechMessage.msg"
  "srv/AudioPlayerByData.srv"
  "srv/SpeechToText.srv"
  "srv/SynthesizeSpeech.srv"
  "srv/AudioPlayer.srv"
  "srv/AudioStreamStart.srv"
  DEPENDENCIES std_msgs audio_common_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
