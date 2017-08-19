function [v] = velocity(oxts,timestamps,e)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%输入oxts：传感器信息；timestamps：时间戳；e：欧拉角
%输出v：速度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g=[0,0,-9.81]';

n=size(timestamps,1);
R_inv = eul2rotm(e(1:3,1))';
for index = 1:n
    acc(1:3,index) = oxts{1,index}(12:14)'; %读入加速度
     R{index} =eul2rotm(e(1:3,index)); %旋转矩阵
    v_n(index) =  oxts{1,index}(7);
    v_e(index) =  oxts{1,index}(8);
    v_u(index) =  oxts{1,index}(11);
end

v_init(1,1)= oxts{1,1}(8);
v_init(2,1)= oxts{1,1}(7);
v_init(3,1)= oxts{1,1}(11);
v(1:3,1) = v_init;
for index = 1:n-1
     t = (timestamps(index+1,1)-timestamps(index,1))*60+(timestamps(index+1,2)-timestamps(index,2));
  % v(1:3,index+1) = v(1:3,index)+ t*(R{index}*(acc(1:3,index+1)-bias)+g);                 %此处可能写错了
     v(1:3,index+1) = v(1:3,index)+ t*(R{index}*acc(1:3,index+1)+g); 
   % v(1:3,index+1) = [oxts{1,index+1}(7:8)';oxts{1,index+1}(11) ];
end


figure
plot(v_n,'r');
hold on
plot(v(2,:),'b');
title('vn');

figure
plot(v_e,'r');
hold on
plot(v(1,:),'b');
title('ve');

figure
plot(v_u,'r');
hold on
plot(v(3,:),'b');
title('vu');
end

