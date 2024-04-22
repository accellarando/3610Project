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

%% Odometry Testing
mOdo = odometry(nb);
% mOdo.turn_degrees(45);
% input("Turned to 45 degrees. Continue?");
% mOdo.turn_degrees(-90);
% input("Turned to -45 degrees. continue?");
% mOdo.turn_degrees(45);
% fprintf("Turned to starting position - done with test.");
%while(1)
    %[r,g,b] = mOdo.get_color()
    %nb.colorRead()
    %input("again?");
%end

mOdo.do_color_task();
