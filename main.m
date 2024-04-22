%%%%%%%%%%
% ECE 3610
% Final Project
%
% Main Program Entry
%%%%%%%%%%

%% Clear the workspace; connect to nanobot
clc;
clear all;
nb = nanobot('/dev/ttyACM0', 115200, 'serial');
%nb.initColor();
mOdo = odometry(nb);

%% Odometry Testing
mOdo.go_straight_distance(-5);
input("Went 5 - continue?");
mOdo.go_straight_distance(5);
input("Went -5 - continue with angle testing?");

%% Angle Testing
mOdo.turn_degrees(45);
input("Turned to 45 degrees. Continue?");
mOdo.turn_degrees(-90);
input("Turned to -45 degrees. continue?");
mOdo.turn_degrees(45);
sprintf("Turned to starting position - done with test.");
while(1)
    [r,g,b] = mOdo.get_color()
    nb.colorRead()
    input("again?");
end

%% Do color task
mOdo.do_color_task();

%% Return manually (gesture control)
gestureNb = nanobot('/dev/ttyACM1', 115200, 'serial');
load('NERD_NETWORK.mat'); %can also just double-click it from the GUI

while(1)
	% DOES NOT TERMINATE! todo: implement line/color detection code to break this while loop?
	% Gesture is performed during the segement below
	gestureNb.ledWrite(0);
	pause(.5);
	countdown("Beginning in", 3);
	disp("Make A Gesture!");
	gestureNb.ledWrite(1); % Begin recording data
	numreads = 100; % about 1.5 seconds (on serial)
	for i = 1:numreads
		val = gestureNb.accelRead();
		vals(1,i) = val.x;
		vals(2,i) = val.y;
		vals(3,i) = val.z;
	end

	gestureNb.ledWrite(0); % Set the LED to red

	rtdata = [vals(1,:);vals(2,:);vals(3,:)];

	% put accelerometer data into NN input form
	xTestLive = zeros(3,100,1,1);
	xTestLive(:,:,1,1) = rtdata;

	% Prediction based on NN
	prediction = classify(myNeuralNetwork,xTestLive)
	string(prediction)

	% Here is where you decide what to do based on gesture
	if(string(prediction) == '1')
		mOdo.set_speeds(-11,8); % should go left
		pause(0.5);
		mOdo.set_speeds(0,0);
	elseif(string(prediction) == '7')
		mOdo.set_speeds(11,-8); % should go right
		pause(0.5);
		mOdo.set_speeds(0,0);
	elseif(string(prediction) == '9')
		mOdo.set_speeds(-11,-8) %should go straight
		pause(1.5);
		mOdo.set_speeds(0,0);
	end
end
