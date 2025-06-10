# Music-Following-Robot
## Learning.c
### Goal 
The goal of `Learning.c` is to build a foundational understanding of how to program the MSP432 microcontroller. This includes learning how the board interprets code, how its internal components function, and how to write basic control logic using bitwise operations. These fundamentals are essential for later stages of the project, specifically, developing a sound-following robot.

To accomplish this, we practice reading analog inputs and preparing them for digital processing by the microcontroller. This involves both signal amplification and proper interpretation of input data through code. Mastering these concepts is a key step toward enabling the robot to react accurately to sound stimuli.

## Final.c
### Goal
The goal of `Final.c` is to implement a complete music-following robot system using the MSP432 microcontroller. This code brings together all components of the project: analog audio signal acquisition, digital filtering, direction decision-making, and motor control.

The system captures stereo audio input from two microphones, filters the signals to isolate sound in the 200 to 3000 Hz range, and compares the amplitudes to determine the direction of the sound source. Based on this analysis, the robot makes real-time navigation decisions such as moving forward, turning left or right, stopping, or performing a 180 degree rotation. This allows it to follow a speaker playing music.

This program demonstrates a practical application of embedded systems principles including ADC sampling, digital signal processing, PWM motor control, and interrupt-driven logic.
