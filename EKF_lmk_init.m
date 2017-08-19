function [lmk,p]=EKF_lmk_init(p,dym,ob,R,rho,rho_var)
%��ʼ��lmkλ��
%����۲�ģ�ͣ�Montiel J M M, Civera J, Davison A J. Unified inverse depth parametrization for monocular SLAM[C]. Robotics: Science and Systems, 2006.
%pЭ�������dym��ǰ������˶�״̬��ob�۲�

    [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);

    [lmk,gr,gy,gs]=inverse_observe(dym,ob',rho) ; %��ʼ��lmk������۲�ģ��
    p_ll = gr*p_xx*gr'+gy*R*gy'+gs*rho_var*gs';           %������lmk����
    p_lx = gr*[p_xx,p_xm];                              %������lmkЭ����
    p=[p,p_lx';
       p_lx,p_ll];                                     %����Э�������
    [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);

%end initialize
end