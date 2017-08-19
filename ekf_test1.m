clear
dbstop if error

load('data');
ob =    data.ob;     %带噪声的观测
acc = data.acc_noise;  %加速度测量值
aw = data.aw_noise;          %角速度测量值

T = 1;  %时间周期
const_step = 300; 
%initialize
init_r = [0 0 0]';                %摄像机位置初始化
init_v = [1,0,0]';   %摄像机速度初始化
init_q = [1,0,0,0]';    %摄像机角度初始化，四元数
init_p = zeros(10,10);                %协方差矩阵初始化，初始状态认为是确定的

Q = [(1e-5)*eye(3),zeros(3,3);
    zeros(3,3),(1e-5)*eye(3)];           %运动方程协方差矩阵
R = [6,0;0,6];                     %观测方程协方差矩阵

rho =0.1;              %lmk的深度
rho_var = 0.05;           %lmk深度的协方差

dym = [init_r;init_v;init_q];                       %当前时刻运动状态初始化

p_xx= init_p;
p_xm=[];
p_mx=[];
p_mm=[];
p=[p_xx,p_xm;
    p_mx,p_mm];                    %p初始化


ob_cell = cell(1,300);
idx_state = 1;
for idx=1:size(ob,1)
   if idx==1
     img_name = ob(idx,1);
     ob_cell{idx_state} = [ob_cell{idx_state};ob(idx,:)];
   else
       if img_name==ob(idx,1)  %是一样的
          ob_cell{idx_state} = [ob_cell{idx_state};ob(idx,:)];
       else
           idx_state = idx_state+1;
           ob_cell{idx_state} = [ob_cell{idx_state};ob(idx,:)];
           img_name = ob(idx,1);
       end
   end
end



table=[];
lmk=[];

for idx_cell=1:size(ob_cell,2)                                   %运动的次数
     [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
     [dym,ft] =  dynamic_predict(dym ,acc(1:3, idx_cell),aw(1:3, idx_cell), T );  %预测，运动方程
     b = Bw(T ,dym(7:10,1));                             %噪声的变换矩阵
     p_xx = ft*p_xx*ft'+b*Q*b';
     if ~isempty(p_xm)
     p_xm=ft*p_xm;
     end
     p=[p_xx,p_xm;
         p_xm',p_mm];
     
    if ~isempty(table)
    for idx = 1:size(ob_cell{idx_cell},1)                             %更新，在table中的观测
        if(~isempty(find(ismember(table(1,:), ob_cell{idx_cell}(idx,1)))))
            [p,lmk,dym]=kf_update( ob_cell{idx_cell}(idx,3:4),p,lmk,dym,R, ob_cell{idx_cell}(idx,2), table);          
        end
    end
    end
    
    for idx = 1:size(ob_cell{idx_cell},1)                              %初始化没有在table中的观测
        if isempty(table)
              [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_cell{idx_cell}(idx,3:4),R, rho, rho_var);
              table = [table [ob_cell{idx_cell}(idx,2);size(p,1)]]; 
        else
            if (isempty(find(ismember(table(1,:), ob_cell{idx_cell}(idx,1))))) && (ob_cell{idx_cell}(idx,2)~=-1)
              [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_cell{idx_cell}(idx,3:4),R, rho, rho_var);
              table = [table [ob_cell{idx_cell}(idx,2);size(p,1)]]; 
            end
        end
    end
    dym_state(1:10,idx_cell)=dym;
    lmk_position{idx_cell}=lmk;
    p_state{idx_cell} = p;
end


estimate.dym_state=dym_state;
estimate.lmk=lmk_position;
estimate.p=p_state;