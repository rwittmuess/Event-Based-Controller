function dpSw = dpSw_gen(in1)
%dpSw_gen
%    dpSw = dpSw_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    03-Mar-2023 17:42:53

dq2 = in1(9,:);
dq3 = in1(10,:);
dx = in1(6,:);
dy = in1(7,:);
q2 = in1(4,:);
q3 = in1(5,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q2+q3+t3;
t5 = cos(t4);
t6 = sin(t4);
dpSw = [dx-dq2.*t6-dq3.*t6;dy-dq2.*t5-dq3.*t5];
