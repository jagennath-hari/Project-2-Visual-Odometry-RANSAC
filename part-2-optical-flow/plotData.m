function plotData(estimatedV, data, Vicon, Time, datasetNum)
%PLOTDATA Plot the predicted data
xVel = estimatedV(1,:);
yVel = estimatedV(2,:);
zVel = estimatedV(3,:);
xAngVel = estimatedV(4,:);
yAngVel = estimatedV(5,:);
zAngVel = estimatedV(6,:);
sampledTime = vertcat(data(:).t);

figure('Name', sprintf('Model %d - Dataset %d', 2, datasetNum));
%%
subplot(2,3,1);
plot(sampledTime, xVel,'r', Time, Vicon(7,:),'b');
title('Velocity X');
legend('Predicted', 'Actual');
subplot(2,3,4);
plot(sampledTime, xAngVel, 'r',Time, Vicon(10,:),'b');
title('Angualr Velocity X');
xlabel('Time (s)');
%%
subplot(2,3,2);
plot(sampledTime, yVel,'r', Time, Vicon(8,:),'b');
title('Velocity Y');
subplot(2,3,5);
plot(sampledTime, yAngVel, 'r',Time, Vicon(11,:),'b');
title('Angualr Velocity Y');
xlabel('Time (s)');

%%
subplot(2,3,3);
plot(sampledTime, zVel,'r', Time, Vicon(9,:),'b');
title('Velocity Z');
subplot(2,3,6);
plot(sampledTime, zAngVel, 'r',Time, Vicon(12,:),'b');
title('Angualr Velocity Z');
xlabel('Time (s)');
end


