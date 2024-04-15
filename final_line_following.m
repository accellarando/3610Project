% Left motor is motor 2, right motor is motor 1
% Sensor numbers go 1 - 6 from left to right
% Initialize robot stuff
clc
clf
clear all
nb = nanobot('COM3', 115200, 'serial');
nb.initReflectance();


% Globals
min_reflectance = [142,106,94,82,94,142];
kp = 0.001;
ki = 0;
kd = 0.0007;
prev_error = 0;
prev_time = 0;
run_time = 40;
integral = 0;
derivative = 0;
max_speed = 10;
motor_speed_offset = 0.1 * max_speed;
all_white_threshold = 300;

% Data collection arrays
times = [];
errors = [];
controls = [];
P_terms = [];
I_terms = [];
D_terms = [];

tic
% To help overcome static friction
nb.setMotor(1, motor_speed_offset);
nb.setMotor(2, motor_speed_offset);
pause(0.03);

counter = 0;
% In cycles
back_up_time = 3000;

% Loop
while (toc < run_time)

    % TIME STEP
    current_time = toc;
    dt = current_time - prev_time;
    prev_time = current_time;

    if counter ~= 0
        fprintf('backing up\n');
        if counter == back_up_time
            counter = 0;
        else
            counter = counter + 1;
        end
    else

    

    % Read sensor values
    valss = nb.reflectanceRead();
    
    vals = [valss.one, valss.two, valss.three, valss.four, valss.five, valss.six];
    calibratedVals = zeros(1,6);
    
    % Calibrate sensor readings, min is 0
    for i = 1:6
        calibratedVals(i) = max(vals(i) - min_reflectance(i), 0);
    end 
   
    % Calculate error, will range from -2500 to 2500
    weighted_sum = dot(calibratedVals, [0, 1000, 2000, 3000, 4000, 5000]);
    error = weighted_sum / sum(calibratedVals) - 2500;

    % Print values of sensors after adjusting
    %fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n',calibratedVals.one, calibratedVals.two, calibratedVals.three, calibratedVals.four, calibratedVals.five, calibratedVals.six);
    fprintf('error: %.2f\n', error);

    % Calculate position error
    if sum(calibratedVals) <= all_white_threshold
        fprintf('All sensors on white\n');
        if error <= 0 
            nb.setMotor(2, -9);
            
        else
            nb.setMotor(1, -9);
        end

        counter = 1;
        continue;
    end
    
    % Calculate PID stuff
    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;
    control = kp * error + kd * derivative;
    fprintf('control: %.2f\n', control);

     % Store data for plotting
    times = [times, current_time];
    errors = [errors, error];
    controls = [controls, control];
    P_terms = [P_terms, kp * error];
    I_terms = [I_terms, ki * integral];
    D_terms = [D_terms, kd * derivative];

    motor1_current_speed = max(min(max_speed - control, max_speed), 0);
    motor2_current_speed = max(min(max_speed + control, max_speed), 0);

    nb.setMotor(2, motor2_current_speed);
    nb.setMotor(1, motor1_current_speed);

    prev_error = error;
    end

end
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Plot PID terms
figure(1);
plot(times, P_terms, 'r', times, I_terms, 'g', times, D_terms, 'b', times, controls, 'k');
legend('P', 'I', 'D', 'Control signal');
title('PID Components and Control Signal');
xlabel('Time (seconds)');
ylabel('Magnitude');

figure(2);
plot(times, errors, 'm');
title('Error');
xlabel('Time (seconds)');
ylabel('Error term');
yline(0);

%% Reset Motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% Tune Motor offset
% clc
% clf
% clear all
% nb = nanobot('COM4', 115200, 'wifi');
% mOffScale = 0.9;
% duty = 10;
% nb.setMotor(1, duty);
% nb.setMotor(2, mOffScale * duty);
% pause(2)
% nb.setMotor(1, 0);
% nb.setMotor(2, 0);

