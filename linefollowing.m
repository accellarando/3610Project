%%%%%%%%%%%%%
% ECE 3610
% LAB 12 -- PID-Based Line Following
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, you will be working in teams to create a PID feedback loop
% and use it to tune a robot to smoothly follow a black line on a white
% background. Line following will be a core part of your final project, so
% it's good to get some experience with it early!
%
% Deliverables:
%   - Demonstrate that your robot accurately and smoothly follows a provided line
%   without losing tracking.
%%%%%%%%%%%%%%

%% WIRING
% This lab has lots of wiring! Be sure to check out the wiring diagram on
% Canvas to map all of the wires where they need to go. Double check this
% before connecting the Arduino to power.

%% 1. CONNECT TO YOUR NANOBOT (SUPER SPECIAL WIFI EDITION)
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

% NOTE: This is the first time we will be connecting to our robot via
% Wi-Fi. Make sure to change string to specify 'wifi' as the third parameter in
% the nanobot() function. Your port should be identical to the one you've
% been using in previous labs.
% Make sure you're connected to the Arduino's wifi network before you
% connect here.

clc
clear all
nb = nanobot('COM4', 115200, 'wifi');

%% RUN IF GOING TO USE CALIBRATED SENSOR VALUES - Calibrate Reflectance Minimums (white background)
minReflectance  = [118, 94, 83, 83, 94, 143]; % Set me to minimum reflectance values for each sensor
nb.initReflectance();
vals = nb.reflectanceRead();
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n', vals.one - minReflectance(1), vals.two - minReflectance(2), vals.three - minReflectance(3), vals.four - minReflectance(4), vals.five - minReflectance(5), vals.six - minReflectance(6));


%% LINE FOLLOWING PID LOOP
% Though PID tuning can be tedious and frustrating at times, the payoff is
% often worth it! A well-tuned PID system can be surprisingly redundant.
% Good luck, and don't hesitate to ask for help if you're stuck.

% IMPORTANT NOTE: When your battery is getting low, the red LEDs on the underside of
% the reflectance array will turn off and you won't be able to read any
% reflectance values. It is likely that your WiFi connection will become
% unstable as well, if not fail completely. If you suspect that your
% battery is dead, grab an instructor's attention and they will swap your
% battery for a fully charged one.

nb.initReflectance();
vals = nb.reflectanceRead();

% NOTE: Your motors may not turn at the same rate when fed the same duty
% cycle, meaning your robot will constantly drift to one side, which can
% make tuning difficult.
% To combat this, you can make a new section such as:
% nb.setMotor(1, duty);
% nb.setMotor(2, mOffScale * duty);
% pause(2)
% nb.setMotor(1, 0);
% nb.setMotor(2, 0);
% to find a factor mOffScale that roughly makes your robot go in a straight
% line. Apply this to the control signal of the stronger/weaker motor to
% even out the speed difference.
%
% Motor offset factor:
mOffScale = 1.2;

% TUNING:
% Start small (ESPECIALLY with the reflectance values, error can range from zero to several thousand!)
% Tip: when tuning kd, it must be the opposite sign of kp to damp
kp = 1;
ki = 1;
kd = -1;

% Basic initialization
vals = 0;
prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;

% Determine a threshold to detect when white is present on all sensors
whiteThresh = 100; % Max value detected for all white

% The base duty cycle "speed" you wish to travel down the line with
% (recommended values are 9 or 10)
motor1BaseSpeed = 8;
motor2BaseSpeed = 8;
runTime = 10;
tic
% It can be helpful to initialize your motors to a fixed higher duty cycle
% for a very brief moment, just to overcome the gearbox force of static
% friction so that lower duty cycles don't stall out at the start.
% (recommendation: 10, with mOffScale if needed)
nb.setMotor(1, 10);
nb.setMotor(2, 10);
pause(0.03);
while (toc < runTime)  % Adjust me if you want to stop your line following earlier, or let it run longer.

    % TIME STEP
    dt = toc - prevTime;
    prevTime = toc;

    vals = nb.reflectanceRead();

    % ///// UNCOMMENT IF YOU WANT TO USE CALIBRATED SENSOR VALUES
    calibratedVals =   struct('one', 0, 'two', 0, 'three', 0, 'four', 0, 'five', 0, 'six', 0);
    % Calibrate sensor readings
    calibratedVals.one = (vals.one - minReflectance(1));
    calibratedVals.two = (vals.two - minReflectance(2));
    calibratedVals.three = (vals.three - minReflectance(3));
    calibratedVals.four = (vals.four - minReflectance(4));
    calibratedVals.five = (vals.five - minReflectance(5));
    calibratedVals.six = (vals.six - minReflectance(6));
    error = 20*calibratedVals.one + 10*calibratedVals.two + 5*calibratedVals.three - 5*calibratedVals.four - 10*calibratedVals.five - 20*calibratedVals.six;
    % if neg, needs to go right
    % if pos, needs to go left
    
    % Designing your error term can sometimes be just as important as the
    % tuning of the feedback loop. In this case, how you define your error
    % term will control how sharp the feedback response is depending on
    % where the line is detected. This is similar to the error term we used
    % in the Sensors Line Detection Milestone.

    % ///// COMMENT OUT LINE BELOW IF USING CALIBRATED SENSOR VALUES
    %error = '?';

    % Calculate P, I, and D terms

    % Set PID

    integral = (prevError + error);

    derivative = (error - prevError) / (runTime - prevTime);

    control = kp * error + ki * integral + kd * derivative; 

    % STATE CHECKING - stops robot if all sensors read white (lost tracking):
    if (vals.one < whiteThresh && ...
            vals.two < whiteThresh && ...
            vals.three < whiteThresh && ...
            vals.four < whiteThresh && ...
            vals.five < whiteThresh && ...
            vals.six < whiteThresh)
        % Stop the motors and exit the while loop
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
        break;
    else
        % LINE DETECTED:
        
        % Remember, we want to travel around a fixed speed down the line,
        % and the control should make minor adjustments that allow the
        % robot to stay centered on the line as it moves.
        m1Duty = 4;
        m2Duty = 4;
       
        % If you're doing something with encoders to derive control, you
        % may want to make sure the duty cycles of the motors don't exceed
        % the maximum speed so that your counts stay accurate.

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
    end

    prevError = error;
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);
%% EMERGENCY MOTOR SHUT OFF
% If this doesn't work, turn off the power switch on your motor carrier
% board.

% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all