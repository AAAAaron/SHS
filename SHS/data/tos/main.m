% 301自测数据----------------求拐点和斜率

%% 电梯数据
% datadir='电梯指纹\F3-F4\';
% datadir='电梯指纹\F5-F7\';
datadir='电梯指纹\F6-F11\';
% datadir='电梯指纹\F7-F10\';
% datadir='电梯指纹\F10-F6\';
% 
%% 扶梯指纹
% datadir='扶梯指纹1\F3-F4\';
% datadir='扶梯指纹1\F4-F3\';
% 

%% 楼梯数据
% datadir='楼梯指纹\F11-F1\';

floor_id=7; %--------------------------------------------------------第4个楼层开始    example:F1,floor_id=2;  F2,floor_id=3;

filename=dir(datadir);
for j=3:size(filename,1)
    piecename=filename(j).name;
    if (isempty(strfind(piecename,'sensor_raw')))==0
        sensorname=piecename;
    end
end

data=csvread([datadir sensorname]);
start=10;
try
t=data(start:end,28);
catch
    t=data(start:end,1);
end
pressure=data(start:end,15);                              %取出气压数据
accxyz=data(start:end,2:4);
acc=(accxyz(:,1).^2+accxyz(:,2).^2+accxyz(:,3).^2).^0.5;  %取出同样采样率的加速度模值数据



height=[4.3,7.8,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3];


floor_rec=floor_bypressure_matrix(t,pressure,floor_id,height,acc);   %第二列为时间，第二列为楼层ID




