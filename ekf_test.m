clear

load('data');
ob =    data.ob;     %带噪声的观测
acc = data.acc_noise;  %加速度测量值
aw = data.aw_noise;          %角速度测量值

T = 1;  %时间周期

%initialize
init_r = [0 0 0]';                %摄像机位置初始化
init_v = [1,0,0]';   %摄像机速度初始化
init_q = [1,0,0,0]';    %摄像机角度初始化，四元数
init_p = zeros(10,10);                %协方差矩阵初始化，初始状态认为是确定的

Q = [(1e-4)*eye(3),zeros(3,3);
    zeros(3,3),(1e-4)*eye(3)];           %运动方程协方差矩阵
R = [6,0;0,6];                     %观测方程协方差矩阵

rho =0.1;              %lmk的深度
rho_var = 0.05;           %lmk深度的协方差

p_xx= init_p;
p_xm=[];
p_mx=[];
p_mm=[];
p=[p_xx,p_xm;
    p_mx,p_mm];                    %p初始化

dym = [init_r;init_v;init_q];                       %当前时刻运动状态初始化

[lmk,p]=EKF_lmk_init(p,dym,ob,R,rho,rho_var);   %lmk初始化
[p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
%end initialize

lmk_state(:,1) = lmk;
dym_state(1:10,1)=dym; 
p_state{1} = p;
%start EKF
for index_state=1:size(ob,1) 
    %predict
    [x,ft] =  dynamic_predict(dym ,acc(1:3,index_state),aw(1:3,index_state),T);  %预测，运动方程
    b = Bw(T,dym(7:10,1));                             %噪声的变换矩阵
    p_xx = ft*p_xx*ft'+b*Q*b';
    p_xm=ft*p_xm;
    p=[p_xx,p_xm;
        p_xm',p_mm];
    %end predict
    
    dym=x;  
    %for iter_index = 1:10
    
    [p,lmk,dym,kh] = kf_update(ob,p,lmk,dym,R,index_state);  %kalman update
    kh_all{index_state}=kh;
    %end
    [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
     %obj在世界中的位置
%     lmk_e1(1:3,index_state+1)=lmk(1:3,1)+m(lmk(4,1),lmk(5,1))/lmk(6,1);
%     lmk_e2(1:3,index_state+1)=lmk(1:3,2)+m(lmk(4,2),lmk(5,2))/lmk(6,2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   dym_state(1:10,index_state+1)=dym;             %存储运动状态
   p_state{index_state+1} = p;                    %存储协方差矩阵
   lmk_state(:,index_state+1) = lmk;
end
%end EKF


%画lmk收敛过程
  figure;
  hold on
    for index_state = 1:size(ob,1)
        for idx_lmk=1:size(ob,2)/2
            lmk_temp = lmk_state(6*idx_lmk-5:6*idx_lmk, index_state);
            lmk_e{idx_lmk}(:,index_state) = lmk_temp(1:3,1)+m(lmk_temp(4,1),lmk_temp(5,1))/lmk_temp(6,1);
            plot3(lmk_e{idx_lmk}(1,index_state),lmk_e{idx_lmk}(2,index_state),lmk_e{idx_lmk}(3,index_state),'*');
            text(lmk_e{idx_lmk}(1,index_state),lmk_e{idx_lmk}(2,index_state),lmk_e{idx_lmk}(3,index_state),num2str(index_state));
        end
    end
    title('obj in 3D world');
    xlabel('x');ylabel('y');zlabel('z');
%end plot

%dym only use acc and aw
dym = dym_state(1:10,1);
dym_state_noekf(1:10, 1) = dym;
for index_state = 1:size(ob,1)-1
    [x,ft] =  dynamic_predict(dym ,acc(1:3,index_state),aw(1:3,index_state),T); 
     dym=x;
     dym_state_noekf(1:10,index_state+1)=dym;
end

%plot
figure('Name', 'simlink_ekf');
hold on
for idx_lmk=1:size(ob,2)/2
     plot3(generate_data.L(1,idx_lmk), generate_data.L(2,idx_lmk), generate_data.L(3,idx_lmk),'o');
end
      plot3(dym_state(1,:),dym_state(2,:),dym_state(3,:),'g.');
      plot3(generate_data.dym_state(1,:), generate_data.dym_state(2,:), generate_data.dym_state(3,:),'r.');
      plot3(dym_state_noekf(1,:),dym_state_noekf(2,:),dym_state_noekf(3,:),'b.');
    for index_state = size(ob,1):size(ob,1)
        for idx_lmk=1:size(ob,2)/2
             plot3(lmk_e{idx_lmk}(1,index_state),lmk_e{idx_lmk}(2,index_state),lmk_e{idx_lmk}(3,index_state),'*');
        end
    end


 xlabel('x');ylabel('y');zlabel('z');
legend('第一个点准确位置','第2个点准确位置','没用ekf的路径','准确路径','ekf得到的路径');



figure
hold on
plot3(dym_state(1,:),dym_state(2,:),dym_state(3,:),'r.');
for index = 1:size(p_state,2)
     confidence_ellip(p_state{1,index}(1:3,1:3),dym_state(1:3,index)');
end
xlabel('x');ylabel('y');zlabel('z');

dym_diff = generate_data.dym_state - dym_state;


%KH矩阵
for idx=1:size(kh_all,2)
    for pt_idx=1:size(kh{1,1},2)
        tr(idx,pt_idx)=trace(kh_all{idx}{pt_idx})
    end
end
