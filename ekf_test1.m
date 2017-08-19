clear
dbstop if error

load('data');
ob =    data.ob;     %�������Ĺ۲�
acc = data.acc_noise;  %���ٶȲ���ֵ
aw = data.aw_noise;          %���ٶȲ���ֵ

T = 1;  %ʱ������

%initialize
init_r = [0 0 0]';                %�����λ�ó�ʼ��
init_v = [1,0,0]';   %������ٶȳ�ʼ��
init_q = [1,0,0,0]';    %������Ƕȳ�ʼ������Ԫ��
init_p = zeros(10,10);                %Э��������ʼ������ʼ״̬��Ϊ��ȷ����

Q = [(1e-4)*eye(3),zeros(3,3);
    zeros(3,3),(1e-4)*eye(3)];           %�˶�����Э�������
R = [6,0;0,6];                     %�۲ⷽ��Э�������

rho =0.1;              %lmk�����
rho_var = 0.05;           %lmk��ȵ�Э����

dym = [init_r;init_v;init_q];                       %��ǰʱ���˶�״̬��ʼ��

p_xx= init_p;
p_xm=[];
p_mx=[];
p_mm=[];
p=[p_xx,p_xm;
    p_mx,p_mm];                    %p��ʼ��

dym_state(1:10,1)=dym;                 %�������е��˶�״̬
p_state{1} = p;

ob_cell = cell(1,300);
idx_state = 1;
for idx=1:size(ob,1)
   if idx==1
     img_name = ob(idx,1);
     ob_cell{idx_state} = [ob_cell{idx_state};ob(idx,:)];
   else
       if img_name==ob(idx,1)  %��һ����
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
         
    if(idx==1)                           %����������ʼ���۲�
        [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_data(idx,2:3),R, rho, rho_var);
         table = [table [ob_data(idx,1);size(p,1)]]; 
        flag=1;
    else                                  
       if(strcmp(img_name,ob_name(idx,:) ))            %�Ƿ�ͬһ��ͼ
           if(isempty(find(ismember(table(1,:), ob_data(idx,1)))))           %�Ƿ��µĹ۲�
               [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_data(idx,2:5),R, rho, rho_var);    %��ʼ��lmk
               table = [table [ob_data(idx,1);size(p,1)]];  
           else
               
               [p,lmk,dym]=kf_update(ob_data(idx,2:5),p,lmk,dym,R,ob_data(idx,1),table);           %�����¹۲�͸���
           end
       else    
         lmk_position{idx_img}=lmk;
         p_state{idx_img} = p;
         [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
         [x,ft] =  dynamic_predict(dym ,acc(1:3, idx_img),aw(1:3, idx_img), t(idx_img) );  %Ԥ�⣬�˶�����
         b = Bw(t(idx_img),dym(7:10,1));                             %�����ı任����
         p_xx = ft*p_xx*ft'+b*Q*b';
         p_xm=ft*p_xm;
         p=[p_xx,p_xm;
         p_xm',p_mm];
         dym=x;
         dym_state(1:10,idx_img+1)=dym;
         if(ob_data(idx,1)~=-1) 
             if(isempty(find(ismember(table(1,:), ob_data(idx,1)))))           %�Ƿ��µĹ۲�
                  [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_data(idx,2:5),R, rho, rho_var);    %��ʼ��lmk
                  table = [table [ob_data(idx,1);size(p,1)]];  
             else
               [p,lmk,dym]=kf_update(ob_data(idx,2:5),p,lmk,dym,R,ob_data(idx,1),table);           %�����¹۲�͸���
             end
         end
         idx_img = idx_img + 1;
       end
       img_name=ob_name(idx,:);
    end   
end