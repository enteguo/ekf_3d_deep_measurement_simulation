clear
dbstop if error

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

dym = [init_r;init_v;init_q];                       %当前时刻运动状态初始化

p_xx= init_p;
p_xm=[];
p_mx=[];
p_mm=[];
p=[p_xx,p_xm;
    p_mx,p_mm];                    %p初始化

dym_state(1:10,1)=dym;                 %保存所有的运动状态
p_state{1} = p;

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

for idx_cell=1:size(ob_cell,2)
    for idx = 1:size(ob_cell{idx_cell},1)
        if(isempty(find(ismember(table(1,:), ob_cell{idx_cell}(idx,1)))))
            [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_cell{idx}(idx,2:3),R, rho, rho_var);
            table = [table [ob_data(idx,1);size(p,1)]]; 
        end
    end
     if(isempty(find(ismember(table(1,:), ob_data(idx,1)))))
        [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_cell{idx}(idx,2:3),R, rho, rho_var);
        table = [table [ob_data(idx,1);size(p,1)]]; 
     end
         
    if(idx==1)                           %刚启动，初始化观测
        [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_data(idx,2:3),R, rho, rho_var);
         table = [table [ob_data(idx,1);size(p,1)]]; 
        flag=1;
    else                                  
       if(strcmp(img_name,ob_name(idx,:) ))            %是否同一张图
           if(isempty(find(ismember(table(1,:), ob_data(idx,1)))))           %是否新的观测
               [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_data(idx,2:5),R, rho, rho_var);    %初始化lmk
               table = [table [ob_data(idx,1);size(p,1)]];  
           else
               
               [p,lmk,dym]=kf_update(ob_data(idx,2:5),p,lmk,dym,R,ob_data(idx,1),table);           %不是新观测就更新
           end
       else    
         lmk_position{idx_img}=lmk;
         p_state{idx_img} = p;
         [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
         [x,ft] =  dynamic_predict(dym ,acc(1:3, idx_img),aw(1:3, idx_img), t(idx_img) );  %预测，运动方程
         b = Bw(t(idx_img),dym(7:10,1));                             %噪声的变换矩阵
         p_xx = ft*p_xx*ft'+b*Q*b';
         p_xm=ft*p_xm;
         p=[p_xx,p_xm;
         p_xm',p_mm];
         dym=x;
         dym_state(1:10,idx_img+1)=dym;
         if(ob_data(idx,1)~=-1) 
             if(isempty(find(ismember(table(1,:), ob_data(idx,1)))))           %是否新的观测
                  [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_data(idx,2:5),R, rho, rho_var);    %初始化lmk
                  table = [table [ob_data(idx,1);size(p,1)]];  
             else
               [p,lmk,dym]=kf_update(ob_data(idx,2:5),p,lmk,dym,R,ob_data(idx,1),table);           %不是新观测就更新
             end
         end
         idx_img = idx_img + 1;
       end
       img_name=ob_name(idx,:);
    end   
end