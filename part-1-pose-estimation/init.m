function [data, vicon, time] = init(dataNum)
%Loading data from the givendataset
data1 = 'studentdata1.mat';
data4 = 'studentdata4.mat';

switch dataNum
    case 1
        allData = load(data1);
    case 4
        allData = load(data4);
end

data = allData.data;
vicon = allData.vicon;
time = allData.time;
end