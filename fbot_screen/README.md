# Package `fbot_screen`

## Overview

The `fbot_screen` is a ROS 2 package developed to display various types of media on a graphical interface.  
It operates through a node that listens to commands on a specific topic to display sentences, images, videos, or even mirror the content of a ROS image topic.

Additionally, the package facilitates visualization by automatically opening Foxglove Studio in the browser, a web tool for ROS data introspection.

---

## How to Run

To start the system, use the launch file `display.launch.py`, which automates the following processes:

1. **Starts the `rosbridge_server`**  
   Creates a WebSocket bridge for external tool connections like Foxglove.

2. **Starts the `display_node`**  
   Runs the main node responsible for media display.

3. **Opens Foxglove Studio**  
   A command is triggered to open Foxglove Studio in the browser, already connected to `rosbridge`.

### Command to execute:

```bash
ros2 launch fbot_screen display.launch.py
```

---

## Features and Usage

After starting, the displayed content is controlled through the `/display_command` topic.  
You must publish messages of type `std_msgs/msg/String` following the format:

```
type:value
```

### Available media types

---

### 1. Display a Sentence (`sentence`)

Generates an image with a white background and displays the given text.  
Use `\n` to insert line breaks.

- **Format:**  
  `sentence:Your text here`

- **Example:**

```bash
ros2 topic pub /display_command std_msgs/msg/String "data: 'sentence:Hello, robot!\nHow are you?'"
```

---

### 2. Display an Image (`image`)

Displays an image file from the full path.

- **Format:**  
  `image:/path/to/your/image.jpg`

- **Example:**

```bash
ros2 topic pub /display_command std_msgs/msg/String "data: 'image:/home/user/photos/fbot.png'"
```

---

### 3. Display a Video (`video`)

Plays a video in a loop until a new command is received.

- **Format:**  
  `video:/path/to/your/video.mp4`

- **Example:**

```bash
ros2 topic pub /display_command std_msgs/msg/String "data: 'video:/home/user/videos/presentation.mp4'"
```

---

### 4. Mirror a Topic (`topic`)

Subscribes to a ROS topic that publishes images (`sensor_msgs/msg/Image`) and mirrors its content on the screen. Useful to display camera feeds.

- **Format:**  
  `topic:/camera_topic_name`

- **Example:**

```bash
ros2 topic pub /display_command std_msgs/msg/String "data: 'topic:/camera/image_raw'"
```

---

## Node Details (`media_display_node`)

| Category             | Description                                                                                 |
|--------------------- |---------------------------------------------------------------------------------------------|
| **Published topics** | `/ui_display` (`sensor_msgs/msg/Image`) - Publishes processed images for the interface      |
| **Subscribed topics**| `/display_command` (`std_msgs/msg/String`) - Receives commands to change the displayed media|
| **Parameters**       | `screen_width` (default: `640`) - Display screen width<br>`screen_height` (default: `480`) -   Display screen height  |
---

