function ft = ft_jacobian(dym,a,w,t)
%�˶����̹���״̬dym���ſɱȾ���
%dym�˶�״̬��a���ٶȣ�w���ٶȣ�tʱ��
q0 = dym(7);  %real
q1 = dym(8);   %image
q2 = dym(9);
q3 = dym(10);
    A = [eye(3),t*eye(3),zeros(3,4);
         zeros(3,3),eye(3),zeros(3,4);
         zeros(4,3), zeros(4,3),eye(4)];
    B = [4*q0*a(1)-2*q3*a(2)+2*q2*a(3), 4*q1*a(1)+2*q2*a(2)+2*q3*a(3), 2*q1*a(2)+2*q0*a(3), -2*q0*a(2)+2*q1*a(3);
        2*q3*a(1)+4*q0*a(2)-2*q1*a(3), 2*q2*a(1)-2*q0*a(3), 2*q1*a(1)+4*q2*a(2)+2*q3*a(3), 2*q0*a(1)+2*q2*a(3)
        -2*q2*a(1)+2*q1*a(2)+4*q0*a(3), 2*q3*a(1)+2*q0*a(2), -2*q0*a(1)+2*q3*a(2), 2*q1*a(1)+2*q2*a(2)+4*q3*a(3)]; 
    C = [zeros(10,6),[t*t/2*B; t*B; t/2*s_func(w)]]; 
    ft = A + C;
end