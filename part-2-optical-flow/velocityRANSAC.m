function [Vel] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    %%

    k = round(log(1-0.99)/log(1-e^3)); %Finding number of iterations
    
    maxInlier = 0; %initalizing max inliers variable
    for i=1:k %for loop for k
        inliers = 0; %setting inliers as 0
        random = randperm(length(optPos),3); %geneating 3 random numbers
        corner = [optPos(random(1,1),:); optPos(random(1,2),:); optPos(random(1,3),:)]; %generating three random keypoints from the input
        xyPos = [corner(1,:); corner(2,:); corner(3,:)]; %taking x y location of the keypints
        v = [optV(2*random(1,1) - 1); optV(2*random(1,1)); optV(2*random(1,2) - 1); optV(2*random(1,2)); optV(2*random(1,3) - 1); optV(2*random(1,3))]; %generating velocity for the same keypoints

        f = []; %initialising f matrix
        %for loop for function same as optical flow
        for j = 1:length(xyPos) 
            pos = transpose([xyPos(j, :) 1]); 
            x = pos(1, 1);
            y = pos(2, 1);
            f1 = [-1/Z(random(j)) 0/Z(random(j)) x/Z(random(j)) x*y -(1+(x)^2) y];
            f2 = [0/Z(random(j)) -1/Z(random(j)) y/Z(random(j)) 1+((y)^2) -x*y -x];
            f = [f; f1; f2];
        end
        V = pinv(f) * v;

        
        for q=1:length(optPos)
            POS = transpose([optPos(q, :) 1]); 
            X = POS(1, 1);
            Y = POS(2, 1);
            F1 = [-1/Z(q) 0/Z(q) X/Z(q) X*Y -(1+(X)^2) Y];
            F2 = [0/Z(q) -1/Z(q) Y/Z(q) 1+((Y)^2) -X*Y -X];
            F = [F1; F2];
            
            Pdot = [optV((2 * q)-1); optV((2 * q))];
            del = norm((F * V) - Pdot); %caluting distance from fitted line 
            if (del <= 0.04) %if that distance is less than the threshold given
                inliers = inliers + 1; %increasing inlier count
            end %end statement
        end
        if(inliers > maxInlier) %if inliers are greaters and max inliers
                maxInlier = inliers; %setting that as max inliers
                vel = V; %returning vel
        end

            
    end
       
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    Vel = vel;
end