function [s] = s_func(x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%��Ԫ��΢�ַ���
%T?rnqvist D. Estimation and detection with applications to navigation[D]. Link?ping University Electronic Press, 2008.
%��ʽ2.21
%��3ά����ת����4��4����
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