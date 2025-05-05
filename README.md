# Voice-Controlled Robot Arm
An ESP32-based robotic arm that responds to voice commands, with recording and playback functionality.

![robo](https://github.com/user-attachments/assets/d9efa57f-e1a2-4aed-a99a-e7812cf25a36)

# Overview
This project uses an ESP32 microcontroller and Edge Impulse for voice recognition to implement a voice-controlled robotic arm. The arm can move in multiple directions and control a gripper in response to verbal commands. It also features motion recording and playback capabilities.

# Hardware Requirements

ESP32 development board

4 servo motors

Microphone module compatible with ESP32 ADC input (I'm using max 4466)

2 push buttons (for mode selection and gripper control)

3 LEDs for state indication (red, green, blue)

1 RGB LED for command feedback

Power supply sufficient for servo operation


# 3d printing

I provide the STL files for the robot structure  in [](3d_files)

Here you can find the gribber 

The Gripper: https://www.thingiverse.com/thing:4394894

### Note that I mount the manipulator in the last joint of the robot, so no need to print the manipulator


# Pin Configuration

| Component         | Pin | Description                         |
|------------------|-----|-------------------------------------|
| Microphone       | 34  | Analog input for voice detection    |
| Mode Button      | 12  | Toggles between operating modes     |
| Grip Button      | 14  | Alternative manual gripper control  |
| LR Servo         | 25  | Controls left-right movement        |
| UD Servo         | 26  | Controls up-down movement           |
| Gripper Servo    | 27  | Controls gripper open/close         |
| RGB LED (Red)    | 21  | Red component of feedback LED       |
| RGB LED (Green)  | 22  | Green component of feedback LED     |
| RGB LED (Blue)   | 23  | Blue component of feedback LED      |
| Free State LED   | 32  | Indicates Free mode (red)           |
| Record State LED | 33  | Indicates Record mode (green)       |
| Loop State LED   | 13  | Indicates Loop mode (blue)          |

# Software Dependencies

Arduino IDE

ESP32 board support package

ESP32Servo library

Edge Impulse SDK (with model named "limt_inferencing")

# Setup Instructions

## Install Required Libraries:

ESP32 board support package

ESP32Servo library

Edge Impulse SDK (the trained model)

## Hardware Assembly:

Connect servos to specified pins (use external power if needed)\

Connect buttons with pull-up resistors

Connect LEDs through appropriate current-limiting resistors

Connect microphone to analog input pin


## Upload Code:

Open the sketch in Arduino IDE

Select your ESP32 board model

Include the edgeimpulse library 

Compile and upload



# Operating Instructions

## The device operates in three modes, toggled by pressing the mode button:

### Free Mode (Red LED):

Voice commands directly control the robot arm
Commands are executed immediately


### Record Mode (Green LED):

Voice commands control the robot arm and record the movements
Recording continues for 10 seconds or until buffer is full
After recording, automatically switches back to Free mode


### Loop Mode (Blue LED):

Replays the previously recorded sequence of movements
After playback completes, remains in Loop mode



# Voice Commands

## Voice Command Actions

| Command | Action              | RGB LED Feedback Color |
|---------|---------------------|-------------------------|
| Left    | Moves arm left      | Red                     |
| Right   | Moves arm right     | Green                   |
| Up      | Moves arm up        | Blue                    |
| Down    | Moves arm down      | Yellow                  |
| Open    | Opens gripper       | Magenta                 |
| Close   | Closes gripper      | Cyan                    |

# Troubleshooting

Servo Jitter: If servos are jittering, check power supply adequacy and increase the detach timeout

Poor Voice Recognition: Retrain the Edge Impulse model with more samples in your usage environment

LEDs Not Working: Verify connections and current-limiting resistors

Unresponsive Buttons: Check debounce implementation and button wiring

# Future Improvements

-Make use of the 3rd servo

-Run multiple modules via SD card 

-Optimize the code more 

-Fix some servo movement issues
