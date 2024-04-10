% Left motor is motor 2, right motor is motor 1
% Sensor numbers go 1 - 6 from left to right
% Initialize robot stuff
clc
clear all
nb = nanobot('COM4', 115200, 'wifi');
nb.initReflectance();

% Globals
last_read_line = 0;
min_reflectance = [142,106,94,82,94,142];
kp = 0.1;
ki = 0.1;
kd = 0.1;
vals = 0;
prev_error = 0;
prev_time = 0;
run_time = 5 ;
integral = 0;
derivative = 0;
motor1_base_speed = 30;
motor2_base_speed = 30;
all_white_threshold = 200;
max_error = 15000;
motor1_current_speed = 0;
motor2_current_speed = 0;

% Loop
tic
% It can be helpful to initialize your motors to a fixed higher duty cycle
% for a very brief moment, just to overcome the gearbox force of static
% friction so that lower duty cycles don't stall out at the start.
% (recommendation: 10, with mOffScale if needed)
nb.setMotor(1, 10);
nb.setMotor(2, 10);
pause(0.03);
while (toc < run_time)

    % TIME STEP
    dt = toc - prev_time;
    prev_time = toc;

    % Read sensor valuesv
    vals = nb.reflectanceRead();
    calibratedVals = struct('one', 0, 'two', 0, 'three', 0, 'four', 0, 'five', 0, 'six', 0);
    
    % % Calibrate sensor readings
    calibratedVals.one = (vals.one - min_reflectance(1));
    calibratedVals.two = (vals.two - min_reflectance(2));
    calibratedVals.three = (vals.three - min_reflectance(3));
    calibratedVals.four = (vals.four - min_reflectance(4));
    calibratedVals.five = (vals.five - min_reflectance(5));
    calibratedVals.six = (vals.six - min_reflectance(6));
    error = calibratedVals.one * 15 + calibratedVals.two * 10 + calibratedVals.three * 5 - calibratedVals.four * 5 - calibratedVals.five * 10 - calibratedVals.six * 15;
    
    % Detect if on white
    if (calibratedVals.one + calibratedVals.two + calibratedVals.three + calibratedVals.four + calibratedVals.five + calibratedVals.six) <= all_white_threshold
        fprintf('off line');
        error = max_error;
        % just stop motors for now
         nb.setMotor(1, 0);
         nb.setMotor(2, 0);
        break;
    end
    
    % Print values of sensors after adjusting
    fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n',calibratedVals.one, calibratedVals.two, calibratedVals.three, calibratedVals.four, calibratedVals.five, calibratedVals.six);
    fprintf('error: %.2f\n',error);
    
    % Calculate P, I, and D terms
    integral = error + prev_error;
    derivative = (error- prev_error) / (run_time - prev_time);

    % Set PID
    control = kp * error + ki * integral + kd * derivative;
    fprintf('control: %.2f\n',control);

    if control >= 0
        motor1_current_speed = motor1_current_speed + motor1_base_speed * (control / max_error);
        motor2_current_speed = motor2_current_speed - motor2_base_speed * (control / max_error);
        if motor1_current_speed >= motor1_base_speed
            motor1_current_speed = motor1_base_speed;
        end
    else
        motor1_current_speed = motor1_current_speed - motor1_base_speed * (control / max_error);
        motor2_current_speed = motor2_current_speed + motor2_base_speed * (control / max_error);
        if motor2_current_speed >= motor2_base_speed
            motor2_current_speed = motor2_base_speed;
        end
    end

    nb.setMotor(2, motor2_current_speed);
    nb.setMotor(1, motor1_current_speed);

    prev_error = error;
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Reset Motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Tune Motor offset
% Motor offset factor:
mOffScale = 1;
duty = 10;
nb.setMotor(1, duty);
nb.setMotor(2, mOffScale * duty);
pause(2)
nb.setMotor(1, 0);
nb.setMotor(2, 0);

