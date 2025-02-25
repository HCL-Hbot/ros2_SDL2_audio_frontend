# Set-up build environment on Linux

Linux is one of the easiest os'es to set-up as most packages and libraries can be found in the package repositories.

## Ubuntu and Debian

We currently support Ubuntu Noble (24.04) 64-bit x86 and 64-bit ARM.

Use [this guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) to install ROS 2 Jazzy Jalisco on your system.

Make sure SDL2 development libraries are installed as well:

```bash
sudo apt-get install libsdl2-dev
```

We also need the ncurses (new curses) library, which is a free software emulation of curses for text-based user interfaces:
```bash
sudo apt-get install libncurses-dev
```

If you have not installed VSCode yet, do not install using APT in ubuntu as it will install the sandboxed snap version.

**Which has many issues due to the sandbox environment**

Use [this guide](https://code.visualstudio.com/docs/setup/linux) instead, which installs it using the APT repository from Microsoft themselves.


## Compiling and running the example

To demonstrate the usage of the ROS2 package, compile and run the nodes:

1. Clone this repo:
```
git clone https://github.com/HCL-Hbot/ros2_SDL2_audio_frontend.git
```

2. Open the cloned repo folder in a terminal


3. Source your ROS2 installation:

```bash
source /opt/ros/jazzy/setup.bash
```

4. Install the ros2 dependencies:
```bash
rosdep install --from-paths src -y --ignore-src
```

5. Build the package with more verbose output

```bash
colcon build --event-handlers console_direct+
```

Note that, whenever build errors occur, and you need to clean the build, use

```bash
rm -rf build install log
rm -rf ~/.ros
```

6. Set up the environment

```bash
source install/setup.bash
```

7. With the environment sourced, we can run executables built by colcon. Let's run using the launch file:

```bash
ros2 launch sdl2_audio_frontend audio_frontend.launch.py
```

8. In another terminal, you can verify that the nodes are publishing to their respective topics (don’t forget to source the setup script):
```bash
ros2 topic list
/audio/raw
/parameter_events
/rosout
/voice_detected
/wake_word_detected
```
You should see at least these topics:
- /audio/raw (from audio_capture_node)
- /voice_detected (from vad_node)
- /wake_word_detected (from wake_word_node)

9. You can check the topic published by the VAD node (don’t forget to source the setup script):
```bash
ros2 topic echo /voice_detected
```

You should see a boolean being published
```
...
---
data: false
---
data: false
...
```
