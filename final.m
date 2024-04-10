% Left motor is motor 2, right motor is motor 1
% Sensor numbers go 1 - 6 from left to right
% Initialize robot stuff
clc
clear all
nb = nanobot('COM4', 115200, 'wifi');
nb.initReflectance();

% Globals
min_reflectance = [142,106,94,82,94,142];
kp = 0.0001;
kd = 0.0004;
vals = 0;
prev_error = 0;
prev_time = 0;
run_time = 10;
derivative = 0;
all_white_threshold = 200;
max_error = 2500;
max_speed = 12;
min_error = -2500;
motor1_current_speed = 0;
motor2_current_speed = 0;

% Loop
tic
% It can be helpful to initialize your motors to a fixed higher duty cycle
% for a very brief moment, just to overcome the gearbox force of static
% friction so that lower duty cycles don't stall out at the start.
% (recommendation: 10, with mOffScale if needed)
nb.setMotor(1, max_speed);
nb.setMotor(2, max_speed);
pause(0.03);
while (toc < run_time)

    % TIME STEP
    dt = toc - prev_time;
    prev_time = toc;

    % Read sensor valuesv
    vals = nb.reflectanceRead();
    calibratedVals = struct('one', 0, 'two', 0, 'three', 0, 'four', 0, 'five', 0, 'six', 0);
    
    % Calibrate sensor readings, min is 0
    if vals.one > min_reflectance(1)
        calibratedVals.one = (vals.one - min_reflectance(1));
    end
    if vals.two > min_reflectance(2)
        calibratedVals.two = (vals.two - min_reflectance(2));
    end
    if vals.three > min_reflectance(3)
        calibratedVals.three = (vals.three - min_reflectance(3));
    end
    if vals.four > min_reflectance(4)
        calibratedVals.four = (vals.four - min_reflectance(4));
    end
    if vals.five > min_reflectance(5)
        calibratedVals.five = (vals.five - min_reflectance(5));
    end
    if vals.six > min_reflectance(6)
        calibratedVals.six = (vals.six - min_reflectance(6));
    end    
   
    % Calculate error, will range from -2500 to 2500
    error = (0 * calibratedVals.one + 1000 * calibratedVals.two + 2000 * calibratedVals.three + 3000 * calibratedVals.four + 4000 * calibratedVals.five + 5000 * calibratedVals.six) / (calibratedVals.one + calibratedVals.two + calibratedVals.three + calibratedVals.four + calibratedVals.five + calibratedVals.six) - 2500;


    % Detect if on white
    if (calibratedVals.one + calibratedVals.two + calibratedVals.three + calibratedVals.four + calibratedVals.five + calibratedVals.six) <= all_white_threshold
        fprintf('off line\n');
        if prev_error < 0 
            error = min_error;
        else
            error = max_error;
        end
        nb.setMotor(2, -9);
        nb.setMotor(1, -9);
        
    else
    
    % Print values of sensors after adjusting
    %fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n',calibratedVals.one, calibratedVals.two, calibratedVals.three, calibratedVals.four, calibratedVals.five, calibratedVals.six);
    fprintf('error: %.2f\n', error);
    
    % Calculate D
    derivative = (error - prev_error) / (run_time - prev_time);

    % Set PID
    control = kp * error + kd * derivative;
    fprintf('control: %.2f\n', control);

    motor1_current_speed = max_speed - control;
    motor2_current_speed = max_speed + control;

    if motor1_current_speed < 0
        motor1_current_speed = 0;
    end
    if motor2_current_speed < 0
        motor2_current_speed = 0;
    end
    if motor1_current_speed > max_speed
        motor1_current_speed = max_speed;
    end
    if motor2_current_speed > max_speed
        motor2_current_speed = max_speed;
    end

    nb.setMotor(2, motor2_current_speed);
    nb.setMotor(1, motor1_current_speed);

    prev_error = error;

    end
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Reset Motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Tune Motor offset
% Motor offset factor:
%mOffScale = 1;
%duty = 10;
%nb.setMotor(1, duty);
%nb.setMotor(2, mOffScale * duty);
%pause(2)
%nb.setMotor(1, 0);
%nb.setMotor(2, 0);

