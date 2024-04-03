# 3610Project
[Documentation](https://docs.google.com/document/d/1FCm-gnY11rz_PwXo4BpYIMVihD4Bvm1cCrFoAE9PToI/edit)

## Line Following
1. Initialize reflectance arrays
2. Read each array, calculate `error = -3*I1 - 2*I2 - I3 + I4 + 2*I5 + 3*I6` 
3. PID controller: `P = Kp*error`, `I = Ki*(error + prev_error)`, `D = Kd*(error - prev_error)`. Sum these terms for control.
4. Tune PID values to get good line following behavior.

## Wall Following
1. Initialize ultrasonic sensors (1 is front, 2 is side)
2. Arrive at vertical black tape section - this marks the start of the round wall-following obstacle.
3. Turn right 90 degrees.
4. Continuously read ultrasonic sensors until the black (floor) tape is detected again (after one loop).

## Odometry - Ella
Needs to drive in a straight line, and turn at a specific angle. We'll probably need a PID loop based on encoder counts.

Encoder conversion: 1440 counts per 1 shaft revolution. 

Wheel diameter: 7cm

Distance between wheels: 15cm

### Speed Capping
Set min and max speeds when doing PID so that it doesn't do anything too crazy

### Straight Line Odometry
Control both motors with PID loops to drive in a straight line. One motor may be stronger: this will be reflected in encoder values. Once you've reached the required distance you're good.

Extension from odometry to a distance: integrate with IR sensors, and stop once you hit the black tape.

### Angular Odometry
Need to keep track of distance between wheels (constant), wheel radius (constant), and encoder counts per revolution (1440). We'll need to derive some sort of equation for this. Also program in some acceptable "margin of error" - once we're close enough, just call it good. No need for oscillations.

### Color Sensing
1. initColor()
2. Detect color card
3. Turn to corresponding color target and go there

## Path Planning and Gesture Recognition
We need to communicate which objective to complete via the stick. 

Maybe ask for the gesture twice to make sure it's received correctly.

These are the easiest gestures for my CNN to distinguish. But if we need more we can add them of course.

| Gesture | Objective |
| --- | --- |
| 0 | Obj |
| 1 | Obj |
| 5 | Obj |
| 9 | Obj |

