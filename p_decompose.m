function [p_xx,p_xm,p_mx,p_mm] = p_decompose(p)
%%把协方差矩阵拆开
%p=[p_xx,p_xm;
%   p_mx,p_mm]
%
    [m,n] = size(p);
    if(m ~= n)
        printf('p matrix dimensionality is error!');
    end
    
    if( m<=10 )
        p_xx = p;
        p_xm=[];
        p_mx = p_xm';
        p_mm = [];
    else
        p_xx = p(1:10,1:10);
        p_xm = p(1:10,11:n);
        p_mx = p_xm';
        p_mm = p(11:m,11:n);
    end
end