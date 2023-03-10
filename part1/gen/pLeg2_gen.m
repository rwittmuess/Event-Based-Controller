function pLeg2 = pLeg2_gen(in1)
%pLeg2_gen
%    pLeg2 = pLeg2_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    03-Mar-2023 17:42:54

q2 = in1(4,:);
q3 = in1(5,:);
x = in1(1,:);
y = in1(2,:);
t2 = pi./2.0;
t3 = -t2;
t4 = q2+q3+t3;
pLeg2 = [x+cos(t4);y-sin(t4)];
