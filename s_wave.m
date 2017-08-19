function [s] = s_wave(q)
%噪声前的S波浪的计算
s = [-q(2),-q(3),-q(4);
    q(1),-q(4),q(3);
    q(4),q(1),-q(2);
    -q(3),q(2),q(1)];
end