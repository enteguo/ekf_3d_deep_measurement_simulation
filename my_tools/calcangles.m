function [e] = calcangles(oxts,timestamps)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%输入oxts：传感器信息，timestamps：时间戳
%输出e：Euler角，三维分别是roll（绕x轴旋转），pitch（绕y轴旋转），yaw（绕z轴旋转）
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% timestamps = importdata('E:\dataset\2011_09_30_drive_0027_sync\2011_09_30\2011_09_30_drive_0027_sync\oxts\timestamps.txt');
% timestamps = timestamps.data;            %时间戳
% oxts = loadOxtsliteData('E:\dataset\2011_09_30_drive_0027_sync\2011_09_30\2011_09_30_drive_0027_sync')%传感器信息，x,y,z方向角速度在（18，19，20）
n = size(timestamps,1);

q_init = eul2qua(oxts{1,1}(4:6));
for index = 1:n
    v_angles(1:3,index) = oxts{1,index}(18:20)'; 
    rpy(1:3,index) = oxts{1,index}(4:6)';
end

q(1:4,1) = q_init; 
e(1:3,1) = oxts{1,1}(4:6)';
for index = 1:n-1
    t = (timestamps(index+1,1)-timestamps(index,1))*60+(timestamps(index+1,2)-timestamps(index,2));
    q(1:4,index+1) = q(1:4,index) + t/2*vec2mat44(v_angles(1:3,index+1))*q(1:4,index);
    e(1:3,index+1) = qua2eul(q(1:4,index+1));
end

figure
plot(rpy(1,:),'r');
hold on
plot(e(1,:),'b');
grid on
title('roll');

figure
plot(rpy(2,:),'r');
hold on
plot(e(2,:),'b');
grid on
title('pitch');

figure
plot(rpy(3,:),'r');
hold on
plot(e(3,:),'b');
grid on
title('yaw');