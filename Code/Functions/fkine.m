% Forward kinematics function
% This function allows to compute forward kinematics for ABB IRB-140
% Input are:
%           1) TE0, transfromation matrix from base frame to EE frame (symbolic matrix)
%           2) q, set of joint variables of the manipulator
% Output is:
%           1) T, transfromation matrix with numeric values
function T = fkine(TE0,q)
    syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t) 
    T = double(subs(TE0,{q1(t),q2(t),q3(t),q4(t),q5(t),q6(t)},{q(1),q(2),q(3),q(4),q(5),q(6)}));
end