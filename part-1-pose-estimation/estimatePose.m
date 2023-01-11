function [position, orientation] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
    
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
    %%
        res = getCorner(data(1,t).id);
        a=[];
        for j =2:2:length(res)
            A = [res(1,j-1) res(1,j) 1 0 0 0 -data(1,t).p0(1,(j/2))*res(1,j-1) -data(1, t).p0(1,(j/2))*res(1,j) -data(1, t).p0(1,(j/2));
                 0 0 0 res(1,j-1) res(1,j) 1 -data(1,t).p0(2,(j/2))*res(1,j-1) -data(1, t).p0(2,(j/2))*res(1,j) -data(1, t).p0(2,(j/2));
                res(2,j-1) res(2,j) 1 0 0 0 -data(1,t).p1(1,(j/2))*res(2,j-1) -data(1, t).p1(1,(j/2))*res(2,j) -data(1, t).p1(1,(j/2));
                0 0 0 res(2,j-1) res(2,j) 1 -data(1,t).p1(2,(j/2))*res(2,j-1) -data(1, t).p1(2,(j/2))*res(2,j) -data(1, t).p1(2,(j/2));
                res(3,j-1) res(3,j) 1 0 0 0 -data(1,t).p2(1,(j/2))*res(3,j-1) -data(1, t).p2(1,(j/2))*res(3,j) -data(1, t).p2(1,(j/2));
                0 0 0 res(3,j-1) res(3,j) 1 -data(1,t).p2(2,(j/2))*res(3,j-1) -data(1, t).p2(2,(j/2))*res(3,j) -data(1, t).p2(2,(j/2));
                res(4,j-1) res(4,j) 1 0 0 0 -data(1,t).p3(1,(j/2))*res(4,j-1) -data(1, t).p3(1,(j/2))*res(4,j) -data(1, t).p3(1,(j/2));
                0 0 0 res(4,j-1) res(4,j) 1 -data(1,t).p3(2,(j/2))*res(4,j-1) -data(1, t).p3(2,(j/2))*res(4,j) -data(1, t).p3(2,(j/2));
                res(5,j-1) res(5,j) 1 0 0 0 -data(1,t).p4(1,(j/2))*res(5,j-1) -data(1, t).p4(1,(j/2))*res(5,j) -data(1, t).p4(1,(j/2));
                0 0 0 res(5,j-1) res(5,j) 1 -data(1,t).p4(2,(j/2))*res(5,j-1) -data(1, t).p4(2,(j/2))*res(5,j) -data(1, t).p4(2,(j/2));];
            a =[a; A];
        end
            [U,S,V] = svd(a);
            h = V(:,9);
            H = transpose(reshape(h, [3,3]));
            H = H./H(3,3);
            Rcap = inv([311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1])*H;
            R1 = Rcap(:,1);
            R2 = Rcap(:,2);
            Tcap = Rcap(:,3);
            [U1,S1,V1] = svd([R1 R2 cross(R1,R2)]);
            R = U1*[1 0 0; 0 1 0; 0 0 det(U1*transpose(V1))]*transpose(V1);
            trans = [0.7071 -0.7071 0 0.04;
                     -0.7071 -0.7071 0 0;
                     0 0 -1 -0.03;
                     0 0 0 1];
            T = (Tcap/norm(R1));
            transMatrix = [R T;
                            0 0 0 1];
            transMatrix = pinv(trans*transMatrix);
            position = [transpose(transMatrix(1:3,end))];
            orientation = [rotm2eul(transMatrix(1:3,1:3))];

end