%% PROJECT 2 VELOCITY ESTIMATION
tic %%starting counter to ensure time is less than 1 min
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.

datasetNum = 4;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
    enableRANSAC = 1; %%RANSAC FLAG CHANGE TO 1 TO RUN WITH RANSAC

    % Rotational matrix of Body wrt Camera frame
    R_cb = [0.707 -0.707 0; -0.707 -0.707 0; 0 0 -1]; %Rotation of camera wrt body frame
    t_cb = [-0.04; 0.0; -0.03];  % Translation of camera wrt body frame
    T_cb =   [R_cb t_cb;0 0 0 1];  % Transformation matrix of body frame wrt camera frame
    T_bc = (T_cb)^-1; % Transformation matrix of camera wrt body
    c = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1]; %Camera interinsic parameters
time = zeros(length(sampledData), 1); %creating new variable for storing time
for i=1:length(time) %looping through time
    time(i,1) = sampledData(1,i).t; %storing sampleData time in variable time
end % end statment
time = sgolayfilt(time, 1, 101); %filtering time using Savitzky-Golay filtering for frame length of 101
%%
for n = 2:length(sampledData) %Main loop
    %% Initalize Loop load images
    imgPrev = im2gray(sampledData(1,n-1).img); %reading previous frame images while converting them to gray scale
    imgCurr = im2gray(sampledData(1,n).img); %reading current frame images while converting them to gray scale
    %% Detect good points
    [featuresPrev, cornersPrev] = extractFeatures(imgPrev, detectORBFeatures(imgPrev)); %Detecting keypoints and descriptors using Shi Tomasi Corner Detector
    cornersPrev = cornersPrev.selectStrongest(100); %Selecting strongest 100 keypoints
    %% Initalize the tracker to the last frame.
    tracker = vision.PointTracker('MaxBidirectionalError',0.5); %Using Kanade-Lucas-Tomasi tracker 
    initialize(tracker,cornersPrev.Location,imgPrev); %Initalizing the point tracker for the previous frame
    %% Find the location of the next points;
    [loc, validity] = tracker(imgCurr); %Getting the translation of the keypoints tracked in the previous frame
    out = insertMarker(imgCurr,loc(validity, :),'+'); %Test code for video
    imshow(out); %Test code for seeing the video 
    corners = []; %corners matrix
    l =[]; % l matrix
    for i=1:length(cornersPrev.Location) %for loop iterating through the corners of the previous frame
        corners = [corners; transpose(inv(c)*transpose([cornersPrev(i,:).Location 1]))]; %Image plane to world frame using camera intrensic parameters
        l = [l; transpose(inv(c)*transpose([loc(i,:) 1]))]; % appending keypoints tracked by chaning those to worl plane using camera intrensic parameters
    end %end statement
    corners = corners(:,1:2); %Removing 3rd coloumn in corners
    loc = l(:,1:2); %Removing 3rd coloumn in loc

    %% Calculate velocity
    % Use a for loop
    v =[]; %intializin a v matrix
    dx = loc(:, :) - corners(:, :); %dx finding dx using loc and locations of corners
    dt = time(n,1)- time(n-1, 1); %finding dt using new filtered time
    vel = dx/dt; %finding linear and angular velocity of those keypoints
    for i=1:length(vel) %iterating through keypoints
        v = [v; vel(i,1); vel(i,2)]; % appending velocity values in v
    end %end statement
    %% Calculate Height
    [position, orientation, R_c2w] = estimatePose(sampledData, n); %using estimate pose to find position, orientation and Rotation matrix from camera to world
    R = eul2rotm(orientation); %Finding rotation matrix
    T_wb = [R position';0 0 0 1]; %Finding the tranformation matrix of world in body frame
    T_cw = (T_wb*T_bc)^-1; %inverse of Tranformation matrix  of world in body frame
    T_wc = T_wb*T_bc; %Tranformation matrix of world in camera frame
    Z = T_cw(3,4);
    z=[]; % empty z matrix

    for zi=1:length(corners) %iterating through length corners
        Z_temp=position(3)/(dot([corners(zi,1);corners(zi,2);1],-1*R_c2w(3,:))); %having a Z_temp variable to find height of every keypoint
        z=[z;Z_temp]; % appening height to z
    end %end statement


    
    %%
    if(enableRANSAC == 0) %if statement for RANSAC flag
        f = []; %initalising a f matrix empty 
        for i = 1:length(loc) %iterating through length of loc
            Z = z(i); %Z is the height for that particular keypoint
            pos = transpose([loc(i, :) 1]); %pos is the loc values
            x = pos(1, 1); %x value 
            y = pos(2, 1); %y value
            f1 = [-1/Z 0/Z x/Z x*y -(1+(x)^2) y]; %function 1
            f2 = [0/Z -1/Z y/Z 1+((y)^2) -x*y -x]; %function 2
            f = [f; f1; f2]; %appending the functions
        end
        Vel = (pinv(f) * v); %calculation velocity by using the psuedo-inverse 
    end %end statement
   

    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    if (enableRANSAC == 1) %if statement for RANSAC flag
        Vel = velocityRANSAC(v, corners, z, R_c2w, 0.42); %Velocity using RANSAC
    end %end statement
    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    r_wc = T_wc(1:3,1:3); %Rotation matrix of world wrt camera frame
    R_bc = T_bc(1:3,1:3); %Rotation matrix of body wrt to camera frame
    Vel = [R zeros(3,3);zeros(3,3) R]*[R_bc -R_bc*skew(T_bc(1:3,4));zeros(3,3) R_bc]*Vel; % Calculation of velocity by changing frames using body twist and adjoint matrix
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended
    estimatedV(:,n) = Vel;
   
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end
%filtering Estimated Velocity using Savitzky-Golay filtering for each
%different cases for applying filter

if(datasetNum == 1 && enableRANSAC == 0)
    estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 19);
    estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 57);
    estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
    estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 5);
    estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 5);
    estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 5);
elseif (datasetNum == 4 && enableRANSAC == 0)
    estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 19);
    estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 25);
    estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 13);
    estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 5);
    estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 5);
    estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 5);
elseif (datasetNum == 1 && enableRANSAC == 1)
    estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 33);
    estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 47);
    estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 25);
    estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 9);
    estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 9);
    estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 9);
elseif (datasetNum == 4 && enableRANSAC == 1)
    estimatedV(1,:) = sgolayfilt(double(estimatedV(1,:)), 1, 17);
    estimatedV(2,:) = sgolayfilt(double(estimatedV(2,:)), 1, 29);
    estimatedV(3,:) = sgolayfilt(double(estimatedV(3,:)), 1, 5);
    estimatedV(4,:) = sgolayfilt(double(estimatedV(4,:)), 1, 9);
    estimatedV(5,:) = sgolayfilt(double(estimatedV(5,:)), 1, 9);
    estimatedV(6,:) = sgolayfilt(double(estimatedV(6,:)), 1, 9);
end


plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
toc %printing time
