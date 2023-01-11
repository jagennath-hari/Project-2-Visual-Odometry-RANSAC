function plotData(Tx, Ty, Tz, eulX, eulY, eulZ, data, Vicon, Time, datasetNum)
%PLOTDATA Plot the predicted data
xPos = Tx;
yPos = Ty;
zPos = Tz;
xOrient =eulX;
yOrient = eulY;
zOrient = eulZ;
sampledTime = vertcat(data(:).t);

figure('Name', sprintf('Model %d - Dataset %d', 1, datasetNum));
%%
subplot(2,3,1);
plot(sampledTime, xPos,'r', Time, Vicon(1,:),'b');
title('Position X');
legend('Predicted', 'Actual');
subplot(2,3,4);
plot(sampledTime, xOrient, 'r',Time, Vicon(4,:),'b');
title('Orientation X');
xlabel('Time (s)');
%%
subplot(2,3,2);
plot(sampledTime, yPos,'r', Time, Vicon(2,:),'b');
title('Position Y');
subplot(2,3,5);
plot(sampledTime, yOrient, 'r',Time, Vicon(5,:),'b');
title('Orientation Y');
xlabel('Time (s)');

%%
subplot(2,3,3);
plot(sampledTime, zPos,'r', Time, Vicon(3,:),'b');
title('Position Z');
subplot(2,3,6);
plot(sampledTime, zOrient, 'r',Time, Vicon(6,:),'b');
title('Orientation Z');
xlabel('Time (s)');
end


