% Left motor is motor 2, right motor is motor 1
% Sensor numbers go 1 - 6 from left to right
% Initialize robot stuff
clc
clear all
nb = nanobot('COM3', 115200, 'serial');
nb.initReflectance();
nb.initUltrasonic1('D2','D3')
nb.initUltrasonic2('D4','D5')
front = nb.ultrasonicRead1();
left = nb.ultrasonicRead2();

% Globals for line following
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
all_black_threshold = 1400;
counter = 0;
back_up_time = 3000; % In cycles

% Globals for wall following
dist = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]; % in cm
val = [210, 296, 348, 486, 701, 842, 881, 993, 1086, 1199, 1310, 1397, 1616, 1631, 1784];
arraySize = size(dist, 2); 
scaleFactorList = zeros(1, arraySize);
for i = 1:arraySize
    % In [units/cm], according to array index
    scaleFactorList(i) = 63;
end
avgScaleFactor = mean(scaleFactorList);
distRange = [.4, 48];


% Main line following loop
tic
% To help overcome static friction
nb.setMotor(1, motor_speed_offset);
nb.setMotor(2, motor_speed_offset);
pause(0.03);
while (toc < run_time)

    % TIME STEP
    current_time = toc;
    dt = current_time - prev_time;
    prev_time = current_time;

    if counter ~= 0
        %fprintf('backing up\n');
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

    % Calculate position error
    if sum(calibratedVals) <= all_white_threshold
        % Check if we are close enough to a wall, maybe put in just main
        % loop if robot gets too close before it detects all black
        frontcm = nb.ultrasonicRead1() / avgScaleFactor;
        if (frontcm < 30)
            % set motors to turn right 90 degrees
            nb.setMotor(1,-10)
            nb.setMotor(2,10)
            leftcm = nb.ultrasonicRead2() / avgScaleFactor;
            % Teep turning until we get a reading from the left ultrasonic
            while leftcm >= 13
                leftcm = nb.ultrasonicRead2() / avgScaleFactor;
                pause(0.1);
            end
            
            % Start moving forward
            nb.setMotor(1, 10);
            nb.setMotor(2, 10);

            % Get out of line following while loop
            counter = 0;
            break;
        end

        % Only gets here if not in front of wall
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

    motor1_current_speed = max(min(max_speed - control, max_speed), 0);
    motor2_current_speed = max(min(max_speed + control, max_speed), 0);

    nb.setMotor(2, motor2_current_speed);
    nb.setMotor(1, motor1_current_speed);

    prev_error = error;
    end

end

% Stop motors breifly before following a wall
nb.setMotor(1, 0);
nb.setMotor(2, 0);

% Start wall following
while (true)

    % Check for line
    valss = nb.reflectanceRead();    
    vals = [valss.one, valss.two, valss.three, valss.four, valss.five, valss.six];
    calibratedVals = zeros(1,6);
    for i = 1:6
        calibratedVals(i) = max(vals(i) - min_reflectance(i), 0);
    end 
    if sum(calibratedVals) > all_black_threshold
        break;
    end


    % Read left ultrasonic sensor
    leftcm =  nb.ultrasonicRead2() / avgScaleFactor;
    % set motor 2 speed, this stays constant as it's closer to wall
    % set motor 1 speed, this will change as it will set the steering rate

    if leftcm > 8
        % set motor 1 speed to increase, this will bring it closer to the
        % wall
        nb.setMotor(2, 6);
    else
        % set motor 1 speed to decrease, this will bring it farther to the
        % wall
        nb.setMotor(1,6);
    end      

    pause(.05);
    nb.setMotor(1, 11);
    nb.setMotor(2, 8);

end

            % set motors to turn right 90 degrees
            nb.setMotor(1,-10)
            nb.setMotor(2,10)

            pause(1);


            % Main line following loop
tic
% To help overcome static friction
nb.setMotor(1, motor_speed_offset);
nb.setMotor(2, motor_speed_offset);
pause(0.03);
while (toc < run_time)

    % TIME STEP
    current_time = toc;
    dt = current_time - prev_time;
    prev_time = current_time;

    if counter ~= 0
        %fprintf('backing up\n');
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

    % Calculate position error
    if sum(calibratedVals) <= all_white_threshold
        % Check if we are close enough to a wall, maybe put in just main
        % loop if robot gets too close before it detects all black
        frontcm = nb.ultrasonicRead1() / avgScaleFactor;
        if (frontcm < 20)
            % set motors to turn right 90 degrees
            nb.setMotor(1,-10)
            nb.setMotor(2,10)
            leftcm = nb.ultrasonicRead2() / avgScaleFactor;
            % Teep turning until we get a reading from the left ultrasonic
            while leftcm >= 13
                leftcm = nb.ultrasonicRead2() / avgScaleFactor;
                pause(0.1);
            end
            
            % Start moving forward
            nb.setMotor(1, 10);
            nb.setMotor(2, 10);

            % Get out of line following while loop
            counter = 0;
            break;
        end

        % Only gets here if not in front of wall
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

    motor1_current_speed = max(min(max_speed - control, max_speed), 0);
    motor2_current_speed = max(min(max_speed + control, max_speed), 0);

    nb.setMotor(2, motor2_current_speed);
    nb.setMotor(1, motor1_current_speed);

    prev_error = error;
    end

end

%% Reset Motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);
