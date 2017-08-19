function [lmk,p]=EKF_lmk_init(p,dym,ob,R,rho,rho_var)
%初始化lmk位置
%反向观测模型，Montiel J M M, Civera J, Davison A J. Unified inverse depth parametrization for monocular SLAM[C]. Robotics: Science and Systems, 2006.
%p协方差矩阵，dym当前摄像机运动状态，ob观测

    [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);

    [lmk,gr,gy,gs]=inverse_observe(dym,ob',rho) ; %初始化lmk，反向观测模型
    p_ll = gr*p_xx*gr'+gy*R*gy'+gs*rho_var*gs';           %计算新lmk方差
    p_lx = gr*[p_xx,p_xm];                              %计算新lmk协方差
    p=[p,p_lx';
       p_lx,p_ll];                                     %增广协方差矩阵
    [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);

%end initialize
end