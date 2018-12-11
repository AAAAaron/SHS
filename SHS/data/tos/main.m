% 301�Բ�����----------------��յ��б��

%% ��������
% datadir='����ָ��\F3-F4\';
% datadir='����ָ��\F5-F7\';
datadir='����ָ��\F6-F11\';
% datadir='����ָ��\F7-F10\';
% datadir='����ָ��\F10-F6\';
% 
%% ����ָ��
% datadir='����ָ��1\F3-F4\';
% datadir='����ָ��1\F4-F3\';
% 

%% ¥������
% datadir='¥��ָ��\F11-F1\';

floor_id=7; %--------------------------------------------------------��4��¥�㿪ʼ    example:F1,floor_id=2;  F2,floor_id=3;

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
pressure=data(start:end,15);                              %ȡ����ѹ����
accxyz=data(start:end,2:4);
acc=(accxyz(:,1).^2+accxyz(:,2).^2+accxyz(:,3).^2).^0.5;  %ȡ��ͬ�������ʵļ��ٶ�ģֵ����



height=[4.3,7.8,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3,4.3];


floor_rec=floor_bypressure_matrix(t,pressure,floor_id,height,acc);   %�ڶ���Ϊʱ�䣬�ڶ���Ϊ¥��ID




