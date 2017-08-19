function [R] = eul2rotm(e)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%欧拉角转换为旋转矩阵
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Rx = [1 0 0; 0 cos(e(1)) -sin(e(1)); 0 sin(e(1)) cos(e(1))]; % base => nav  (level oxts => rotated oxts)
  Ry = [cos(e(2)) 0 sin(e(2)); 0 1 0; -sin(e(2)) 0 cos(e(2))]; % base => nav  (level oxts => rotated oxts)
  Rz = [cos(e(3)) -sin(e(3)) 0; sin(e(3)) cos(e(3)) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)
  
  R  = Rz*Ry*Rx;
end