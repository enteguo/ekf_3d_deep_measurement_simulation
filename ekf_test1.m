clear
dbstop if error

load('data');
ob =    data.ob;     %�������Ĺ۲�
acc = data.acc_noise;  %���ٶȲ���ֵ
aw = data.aw_noise;          %���ٶȲ���ֵ

T = 1;  %ʱ������
const_step =40;%data.const_step;   %�ܵĲ���
%initialize
init_r = [0 0 0]';                %�����λ�ó�ʼ��
init_v = [1,0,0]';   %������ٶȳ�ʼ��
init_q = [1,0,0,0]';    %������Ƕȳ�ʼ������Ԫ��
init_p = zeros(10,10);                %Э��������ʼ������ʼ״̬��Ϊ��ȷ����

Q = [(1e-6)*eye(3),zeros(3,3);
    zeros(3,3),(1e-6)*eye(3)];           %�˶�����Э�������
R = [20,0;0,20];                     %�۲ⷽ��Э�������

rho =0.1;              %lmk�����
rho_var = 0.05;           %lmk��ȵ�Э����

dym = [init_r;init_v;init_q];                       %��ǰʱ���˶�״̬��ʼ��

p_xx= init_p;
p_xm=[];
p_mx=[];
p_mm=[];
p=[p_xx,p_xm;
    p_mx,p_mm];                    %p��ʼ��


ob_cell = cell(1,const_step);
idx_state = 1;
for idx=1:size(ob,1)
    if idx_state<const_step
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
end



table=[];
lmk=[];

for idx_cell=1:size(ob_cell,2)                                   %�˶��Ĵ���
     [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
     [dym,ft] =  dynamic_predict(dym ,acc(1:3, idx_cell),aw(1:3, idx_cell), T );  %Ԥ�⣬�˶�����
     b = Bw(T ,dym(7:10,1));                             %�����ı任����
     p_xx = ft*p_xx*ft'+b*Q*b';
     if ~isempty(p_xm)
     p_xm=ft*p_xm;
     end
     p=[p_xx,p_xm;
         p_xm',p_mm];
     
    if ~isempty(table)
    for idx = 1:size(ob_cell{idx_cell},1)                             %���£���table�еĹ۲�
        if(~isempty(find(ismember(table(1,:), ob_cell{idx_cell}(idx,2)))))
            [p,lmk,dym]=kf_update( ob_cell{idx_cell}(idx,3:4),p,lmk,dym,R, ob_cell{idx_cell}(idx,2), table);          
        end
    end
    end
    
    for idx = 1:size(ob_cell{idx_cell},1)                              %��ʼ��û����table�еĹ۲�
        if (isempty(table))
            if(ob_cell{idx_cell}(idx,2)~=-1)
              [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_cell{idx_cell}(idx,3:4),R, rho, rho_var);
              table = [table [ob_cell{idx_cell}(idx,2);size(p,1)]]; 
            end
        else
            if (isempty(find(ismember(table(1,:), ob_cell{idx_cell}(idx,2))))) && (ob_cell{idx_cell}(idx,2)~=-1)
              [lmk,p]=EKF_lmk_init(lmk,p,dym,ob_cell{idx_cell}(idx,3:4),R, rho, rho_var);
              table = [table [ob_cell{idx_cell}(idx,2);size(p,1)]]; 
            end
        end
    end
    dym_state(1:10,idx_cell)=dym;
    lmk_position{idx_cell}=lmk;
    p_state{idx_cell} = p;
end

lmk_e = zeros(3,size(table,2));
for idx_lmk = 1:size(table,2)
    lmk_temp = lmk(6*idx_lmk-5:6*idx_lmk,1);
    lmk_e(:,table(1,idx_lmk)) = lmk_temp(1:3,1)+m(lmk_temp(4,1),lmk_temp(5,1))/lmk_temp(6,1);
end
% %plot pt in 3d world
%  figure;
%   hold on
%     for index_state = 1:size(table,2)
%             lmk_temp = lmk_state(6*idx_lmk-5:6*idx_lmk, index_state);
%             lmk_e{idx_lmk}(:,index_state) = lmk_temp(1:3,1)+m(lmk_temp(4,1),lmk_temp(5,1))/lmk_temp(6,1);
%             plot3(lmk_e{idx_lmk}(1,index_state),lmk_e{idx_lmk}(2,index_state),lmk_e{idx_lmk}(3,index_state),'*');
%             text(lmk_e{idx_lmk}(1,index_state),lmk_e{idx_lmk}(2,index_state),lmk_e{idx_lmk}(3,index_state),num2str(index_state));
%         end
%     end
%     title('obj in 3D world');
%     xlabel('x');ylabel('y');zlabel('z');
% %end plot

for idx=1:size(lmk_e,2)%%%lmk���ƺ���ʵֵ�Ĳ�ֵ
if(sum(lmk_e(:,idx))==0)
lmk_res(:,idx)=[0,0,0]';
else
lmk_res(:,idx)=  lmk_e(:,idx)-data.lmk(:,idx);
end
end

estimate.dym_state=dym_state;
estimate.lmk=lmk_position;
estimate.p=p_state;
estimate.lmk_e = lmk_e;
save estimate estimate

figure
hold on
plot3(dym_state(1,:), dym_state(2,:), dym_state(3,:));
plot3(lmk_e(1,:), lmk_e(2,:),lmk_e(3,:),'.');