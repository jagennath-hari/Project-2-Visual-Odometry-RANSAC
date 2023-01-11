%% Script for Project 2.
tic
clear all;
close all;
clc;
addpath('../data') 
%Change the number to work on both Dataset 1 and Dataset 4 (Do not use Dataset 9)
datasetNum = 4;
[data, vicon, time] = init(datasetNum);
%% Creating variables for storing computed values

eulX = zeros(1, length(data));
eulY = zeros('like', eulX);
eulZ = zeros('like', eulX);
Tx = zeros('like', eulX);
Ty = zeros('like', eulX);
Tz = zeros('like', eulX);

%% The main component of the code. Everything should be computed in the estimatePose function.

for t=1:length(data) 
    [position, orientation] = estimatePose(data,t);
    Tx(t) = position(1);
    Ty(t) = position(2);
    Tz(t) = position(3);
    eulZ(t) = orientation(1);
    eulY(t) = orientation(2);
    eulX(t) = orientation(3);
end

%% Passing the required values to the Plot Function
plotData(Tx, Ty, Tz, eulX, eulY, eulZ, data, vicon, time, datasetNum)
toc