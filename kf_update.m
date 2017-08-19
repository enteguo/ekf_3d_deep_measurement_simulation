function [p,lmk,dym,kh]=kf_update(ob,p,lmk,dym,R,pt_idx,table)
       idx=find(ismember(table(1,:),pt_idx));
       p_idx=table(2,idx);
       lmk_index = lmk(12*idx-11:12*idx-6,:);
       [ms ,jx,jl]= measurement(lmk_index ,dym);                  %�����Ĺ���
       z = ob- ms;                              %����ֵ������Ĺ��Ʋ�ֵ
        [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
       p_z=[p_xx,p(1:10,p_idx-11:p_idx-6);            
            p(p_idx-11:p_idx-6, 1:10), p(p_idx-11:p_idx-6, p_idx-11:p_idx-6)];
       z_var = [jx,jl]*p_z*[jx,jl]'+R;                                     %��ֵ�ķ������
       p_k=[p(:,1:10),p(:, p_idx-11:p_idx-6)];
       K=p_k*[jx,jl]'/z_var;
       temp = [dym;lmk]+K*z;
       dym=temp(1:10,:);
       lmk = temp(11:size(temp),:);
       p=p-K*z_var*K'; 
end