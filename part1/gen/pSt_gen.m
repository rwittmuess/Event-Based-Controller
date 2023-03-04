function pst = pSt_gen(in1)
%pSt_gen
%    PST = pSt_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    03-Mar-2023 17:42:53

q1 = in1(3,:);
q3 = in1(5,:);
x = in1(1,:);
y = in1(2,:);
t2 = pi.*(3.0./2.0);
t3 = -t2;
t4 = q1+q3+t3;
pst = [x-cos(t4);y+sin(t4)];
