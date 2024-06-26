%% Wall Following

clc
clear all
nb = nanobot('/dev/cu.usbmodem2101', 115200, 'serial');
nb.setMotor(1,0);
nb.setMotor(2,0);

%% Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
nb.initUltrasonic1('D2','D3')
nb.initUltrasonic2('D4','D5')
front = nb.ultrasonicRead1();
left = nb.ultrasonicRead2();

%Take a single ultrasonic reading
fprintf('Front dist = %i   Left dist = %i\n', front, left);

% motor 1
% motor 2


%% variables and such
dist = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]; % in cm
val = [210, 296, 348, 486, 701, 842, 881, 993, 1086, 1199, 1310, 1397, 1616, 1631, 1784];
arraySize = size(dist, 2); 
scaleFactorList = zeros(1, arraySize);
for i = 1:arraySize
    scaleFactorList(i) = 63; % In [units/cm], according to array index
end
avgScaleFactor = mean(scaleFactorList);
distRange = [.4, 48];

% LE CODE

%murphy line following

leftcm = left / avgScaleFactor;
fprintf("Last read: %0.1f cm\n", leftcm);


frontcm = front / avgScaleFactor;
fprintf("Last read: %0.1f cm\n", frontcm);


frontcm = nb.ultrasonicRead1() / avgScaleFactor;
fprintf('Front dist = %i   Left dist = %i\n', frontcm, leftcm);

%pause(.1);
if (frontcm < 20)
    % set motors to turn right 90 degrees
    nb.setMotor(1,-10)
    nb.setMotor(2,10)
    leftcm = nb.ultrasonicRead2() / avgScaleFactor;
    %if left <= 10
    while leftcm >= 13
        leftcm = nb.ultrasonicRead2() / avgScaleFactor;
        pause(0.1);
    end

    nb.setMotor(1, 10);
    nb.setMotor(2, 10);
end


while (true)
    
    leftcm =  nb.ultrasonicRead2()/avgScaleFactor;
    fprintf("Last read: %0.1f cm\n", leftcm);
   

    % set motor 2 speed, this stays constant as it's closer to wall
    % set motor 1 speed, this will change as it will set the steering rate

    if leftcm > 8
        % set motor 1 speed to increase, this will bring it closer to the
        % wall
        nb.setMotor(2, 8);

    else
        % set motor 1 speed to decrease, this will bring it farther to the
        % wall
        nb.setMotor(1,8);
    end      

    pause(.01);
    nb.setMotor(1, 11);
    nb.setMotor(2, 10);

end