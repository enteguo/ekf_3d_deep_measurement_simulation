function [b] = Bw(t,q)
%噪声前的系数矩阵
    b=[t^2/2*eye(3),zeros(3,3);
        t*eye(3),zeros(3,3);
        zeros(4,3),t/2*s_wave(q)];
end