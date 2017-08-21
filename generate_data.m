clear
dbstop if error

%%%%%%%%%%%%%%%%%%%%%%参数设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
const_step= 100;
n=1:1:const_step;
K=[7.215377e+02,0.000000e+00,6.095593e+02;            %内参矩阵
    0.000000e+00 7.215377e+02 1.728540e+02;
    0.000000e+00 0.000000e+00 1.000000e+00];

R_i2c =[ 0.0083,-0.9999,0.0142;                      %从IMU到cam旋转
         0.0128,-0.0141,-0.9998;
         0.9999,0.0085,0.0127];

r_i2c=[-0.3292, 0.7116, -1.0898]';                     %从IMU到cam位移

q_init = [1,0,0,0]';                         %旋转初始化
v_init = [1,0,0]';                           %速度初始化
r_init = [0,0,0]';                           %位移初始化

T = 1;                                       %时间周期
g=[0,0,9.81]';
aw = zeros(3,const_step); 
%%运动有关的矩阵
A = [eye(3),T*eye(3),zeros(3,4);           
    zeros(3,3),eye(3),zeros(3,4);
    zeros(4,3), zeros(4,3),eye(4)];
B = [T^2/2*eye(3),zeros(3,4);
    T*eye(3),zeros(3,4);
    zeros(4,3),T/2*eye(4)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for idx=1:const_step 
    if idx<=100
        acc(:,idx) = [0,0,9.81]';
    elseif idx<=200
        acc(:,idx) = [-sin((idx-100)*pi/200)*v_init(1,:)^2*pi/200,cos((idx-100)*pi/200)*v_init(1,:)^2*pi/200,9.81]';
    else
        acc(:,idx) = [0,0,9.81]';
    end
end
data.acc = acc;

dym = [r_init;v_init;q_init]; 
for idx = 1:const_step
    dym_state(1:10,idx) = dynamic_predict(dym ,acc(:,idx),aw(:,idx),T);
    dym = dym_state(1:10,idx);
end 
data.dym_state = dym_state;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%生成目标点三维位置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
const_pt=30;
for idx=1:const_pt
    if idx<=30
        lmk(1,idx) = rand(1)*95+5;
        lmk(2,idx) = rand(1)*4-2;
        lmk(3,idx) = rand(1)*4-2;
    elseif idx<= 60
        lmk(2,idx) = rand(1)*63;
        lmk(1,idx) =100+sqrt( (rand(1)*5+65)^2-(62-lmk(2,idx))^2) ;
        lmk(3,idx) = rand(1)*5;
    else
        lmk(1,idx) = rand(1)*10+167;
        lmk(2,idx) = rand(1)*100+63;
        lmk(3,idx) = rand(1)*5;
    end
end
figure
hold on
plot(dym_state(1,:),dym_state(2,:))
plot3(lmk(1,:),lmk(2,:),lmk(3,:),'.')
data.lmk=lmk;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%生成观测%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%无噪声观测ob_nonoise
idx_ob=0;
flag = 0;
for idx=1:const_step
    for idx_pt = 1:const_pt
        distance = (lmk(1,idx_pt)-dym_state(1,idx))^2+ (lmk(2,idx_pt)-dym_state(2,idx))^2+ (lmk(3,idx_pt)-dym_state(3,idx))^2;
            if   (dym_state(1,idx)< lmk(1,idx_pt)) &&(distance<400)
               
                hc = K*R_i2c*(qua2rotm(dym_state(7:10,idx))*(lmk(:,idx_pt)-dym_state(1:3,idx))-r_i2c);
                h = [hc(1)/hc(3),hc(2)/hc(3)];
                if (h(1)>=0) && (h(1)<=1242) &&(h(2)>=0) &&(h(2)<=375)
                %if (h(1)>=0) &&(h(2)>=0) 
                    idx_ob= idx_ob+1;
                    ob_nonoise(idx_ob,1) = idx;
                    ob_nonoise(idx_ob,2) = idx_pt;
                    ob_nonoise(idx_ob,3:4) = h;
                flag=1;
                end
            end      
    end
    if flag==0
                idx_ob= idx_ob+1;
                ob_nonoise(idx_ob,1) = idx;
                ob_nonoise(idx_ob,2) = -1;
                ob_nonoise(idx_ob,3:4) = [-1,-1];
           
    end
    flag =0;
end

%观测带噪声
for idx = 1:size(ob_nonoise,1)
    if(ob_nonoise(idx,2)~=-1)
        ob(idx,1:2) = ob_nonoise(idx,1:2);
        ob(idx,3:4) = ob_nonoise(idx,3:4) + normrnd(0,5,1,2);
        while ~((ob(idx,3)>=0) && (ob(idx,3) <=1242) &&(ob(idx,4) >=0) && (ob(idx,4)<=375))
          ob(idx,3:4) = ob_nonoise(idx,3:4) + normrnd(0,5,1,2);
        end
    else
        ob(idx,:) = ob_nonoise(idx,:);
    end
end
% ob(:,1:2) = ob_nonoise(:,1:2);
% ob(:,3:4) = ob_nonoise(:,3:4) + normrnd(0,5,size(ob_nonoise,1),2);
data.ob_nonoise = ob_nonoise;
data.ob = ob;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%给惯性传感器加噪声
acc_noise = acc + normrnd(0,0.00001,size(acc,1),size(acc,2)); %测量加速度，有噪声
aw_noise = aw + normrnd(0,0.00001,size(aw,1),size(aw,2));%测量角速度，有噪声

data.acc_noise = acc_noise;
data.aw_noise = aw_noise;
data.const_step =const_step;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save data data