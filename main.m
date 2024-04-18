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

%% Odometry Testing
mOdo = odometry(nb);
mOdo.go_straight_distance(10);
fprintf("Went forward 10");
pause(1);
mOdo.go_straight_distance(-1);
fprintf("Went backward 1");
pause(1);
mOdo.turn_degrees(45);
fprintf("Turned to 45 degrees");
pause(1);
mOdo.turn_degrees(-90);
fprintf("Turned to -45 degrees");
pause(1);
mOdo.turn_degrees(45);
fprintf("Turned to starting position - done with test.");
