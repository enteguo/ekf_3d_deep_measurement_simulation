function [s] = s_func(x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%四元数微分方程
%T?rnqvist D. Estimation and detection with applications to navigation[D]. Link?ping University Electronic Press, 2008.
%公式2.21
%把3维向量转换成4×4矩阵
%x = [x1,x2,x3];
%s = [0 -x1 -x2 -x3;
%     x1 0 x3 -x2;
%     x2 -x3 0 x1;
%     x3 x2 -x1 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = [0 -x(1) -x(2) -x(3);
    x(1) 0 x(3) -x(2);
    x(2) -x(3) 0 x(1);
    x(3) x(2) -x(1) 0];

end