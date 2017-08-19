function [L,gr,gy,gs]=inverse_observe(dym,ob,d)
%反向观测模型
%输入dym,(r,v,q)摄像机状态
%输入ob，摄像机观测（y1,y2）
%输入d，深度的倒数

K=[7.215377e+02,0.000000e+00,6.095593e+02;
    0.000000e+00 7.215377e+02 1.728540e+02;
    0.000000e+00 0.000000e+00 1.000000e+00];

R_i2c =[ 0.0083,-0.9999,0.0142;    %从IMU到cam旋转
         0.0128,-0.0141,-0.9998;
         0.9999,0.0085,0.0127];

% R_i2c =[0.000998747205656553,-0.999990382071041,0.00425937848763518;
%         0.00841690182755742,-0.00425082113606946,-0.999955569675327;
%         0.999964048696658,0.00103455328422636,0.00841257520873259];
    
syms p1 p2 p3 v1 v2 v3 q1 q2 q3 q4 s y1 y2;
r=[p1,p2,p3].';
v=[v1,v2,v3].';
q=[q1,q2,q3,q4].';
motion_state=[r;v;q];
y=[y1,y2].';

hw = qua2rotm(q).'*R_i2c'*inv(K)*[y;1];
% a = atan2(hw(1),hw(3));
% b = atan2(-hw(2),sqrt(hw(1)^2+hw(3)^2));
% a = atan2(hw(2),hw(1));
% b = atan2(sqrt(hw(1)^2+hw(3)^2),hw(3));
a = atan2(hw(1),hw(3));
b = atan2(-hw(2),sqrt(hw(1)^2+hw(3)^2));
% a = atan(sqrt(hw(1)^2+hw(2)^2)/hw(3));
% b = atan(hw(2)/hw(1));

L= [r.',a,b,s].';
gr = jacobian(L,motion_state.');
gy = jacobian(L,y.');
gs = jacobian(L,s);

L =double( subs(L,{p1,p2,p3,v1,v2,v3,q1,q2,q3,q4,s,y1,y2},{dym(1),dym(2),dym(3),dym(4),dym(5),dym(6),dym(7),dym(8)...
    dym(9),dym(10),d,ob(1),ob(2)}));
gr =double( subs(gr,{p1,p2,p3,v1,v2,v3,q1,q2,q3,q4,s,y1,y2},{dym(1),dym(2),dym(3),dym(4),dym(5),dym(6),dym(7),dym(8)...
    dym(9),dym(10),d,ob(1),ob(2)}));
gy = double(subs(gy,{p1,p2,p3,v1,v2,v3,q1,q2,q3,q4,s,y1,y2},{dym(1),dym(2),dym(3),dym(4),dym(5),dym(6),dym(7),dym(8)...
    dym(9),dym(10),d,ob(1),ob(2)}));
gs =double( subs(gs,{p1,p2,p3,v1,v2,v3,q1,q2,q3,q4,s,y1,y2},{dym(1),dym(2),dym(3),dym(4),dym(5),dym(6),dym(7),dym(8)...
    dym(9),dym(10),d,ob(1),ob(2)}));
end