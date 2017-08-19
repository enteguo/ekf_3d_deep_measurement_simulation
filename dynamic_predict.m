function [ x,ft ] = dynamic_predict(dym ,acc,aw,t )
%摄像机状态预测
%输入上一个状态dym
%输入控制acc,aw
%时间t
%输出摄像机下一个状态x，雅可比矩阵ft
% syms p1 p2 p3 v1 v2 v3 q1 q2 q3 q4;
% p = [p1 p2 p3].';
% v = [v1 v2 v3].';
% q = [q1 q2 q3 q4].';
% state = [p ;v ;q];
g=[0,0,9.81]';
%g=[0,0,-9.80]';
A = [eye(3),t*eye(3),zeros(3,4);
    zeros(3,3),eye(3),zeros(3,4);
    zeros(4,3), zeros(4,3),eye(4)];

B = [t^2/2*eye(3),zeros(3,4);
    t*eye(3),zeros(3,4);
    zeros(4,3),t/2*eye(4)];

x = A*dym+B*[qua2rotm(dym(7:10,1))'*acc-g;s_func(aw)*dym(7:10,1)]; 
 
%x = A*state+B*[qua2rotm(q).'*acc+g;vec2mat44(aw)*q];
% ft = jacobian(x,state.');   %雅可比矩阵
% ft = double(subs(ft,state.',dym'));
% x= double(subs(x,state.',dym'));
ft = ft_jacobian(dym,acc,aw,t);
end