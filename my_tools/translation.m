function [p] = translation(v,e,oxts,timestamps)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%输入v：速度，e:Euler角，oxts：传感器信息，timestamps：时间戳
%输出p：平移
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pose = convertOxtsToPose(oxts);

g=[0,0,-9.81]';

n=size(timestamps,1);
%R_inv = eul2rotm(e(1:3,1))';
for index = 1:n
    acc(1:3,index) = oxts{1,index}(12:14)'; %读入加速度
     R{index} =  eul2rotm(e(1:3,index));
    x(index) = pose{1,index}(1,4);
    y(index) = pose{1,index}(2,4);
    z(index) = pose{1,index}(3,4); 
end

p(1:3,1)=[0,0,0]';%初始位置

bias= 0.005*[1,1,1]';
%p_test(1:3,1)=[0,0,0]';
for index = 1:n-1
    t = (timestamps(index+1,1)-timestamps(index,1))*60+(timestamps(index+1,2)-timestamps(index,2));
   % p(1:3,index+1) = p(1:3,index) + t*v(1:3,index) + t^2/2*(R{index}*(acc(1:3,index+1)-bias)+g);                  %此处可能写错了
    p(1:3,index+1) = p(1:3,index) + t*v(1:3,index) + t^2/2*(R{index}*(acc(1:3,index+1))+g);     
end

for index = 1:n
    p(1:3,index) =  R{1}'*p(1:3,index);
end
figure
plot(p(1,:),'b');
hold on
plot(x,'r');
title('x,pose');

figure
plot(p(2,:),'b');
hold on
plot(y,'r');
title('y,pose');

figure
plot(p(3,:),'b');
hold on
plot(z,'r');
title('z,pose');

figure
plot3(p(1,:),p(2,:),p(3,:),'b');
hold on
plot3(x,y,z,'r');
title('traj');
end