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
mOdo.go_straight_distance(30);
input("Went 30cm. Continue?")
mOdo.go_straight_distance(-30);
input("Went -30 cm. Continue?")
mOdo.turn_degrees(45);
input("Turned to 45 degrees. Continue?");
mOdo.turn_degrees(-90);
input("Turned to -45 degrees. continue?");
mOdo.turn_degrees(45);
fprintf("Turned to starting position - done with test.");
