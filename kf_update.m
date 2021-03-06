function [p,lmk,dym,kh]=kf_update(ob,p,lmk,dym,R,pt_idx,table)
       idx=find(ismember(table(1,:),pt_idx));
       p_idx=table(2,idx);
       lmk_estimate = lmk(6*idx-5:6*idx,:);
       [ms ,jx,jl]= measurement(lmk_estimate ,dym);                  %测量的估计
       z = ob' - ms;                              %测量值与测量的估计差值
        [p_xx,p_xm,p_mx,p_mm] = p_decompose(p);
       p_z=[p_xx,p(1:10,p_idx-5:p_idx);            
            p(p_idx-5:p_idx, 1:10), p(p_idx-5:p_idx, p_idx-5:p_idx)];
       z_var = [jx,jl]*p_z*[jx,jl]'+R;                                     %差值的方差矩阵
       p_k=[p(:,1:10),p(:, p_idx-5:p_idx)];
       K=p_k*[jx,jl]'/z_var;
       temp = [dym;lmk]+K*z;
       dym=temp(1:10,:);
       lmk = temp(11:size(temp),:);
       p=p-K*z_var*K'; 
end