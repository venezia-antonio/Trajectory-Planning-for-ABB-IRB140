% This script generate the ABB IRB140 manipulator transformation matrices 
% using also the Peter Corke Toolbox, useful for debugging
%% Transformation matrices 
syms q1(t) q2(t) q3(t) q4(t) q5(t) q6(t)
global L0 L1 L2 L3 L4 Le

% Length of each link of ABB IRB-140 robot manipulator
L0=0.07;
L1=0.3520;
L2=0.360;
L3=0.380;
L4=0.046;
Le = 0.16;

% Transformation matrices 
T00 = eye(4);
T10 = (DH(q1(t),L1,L0,sym(pi)/2));
T21 = (DH(q2(t),0,L2,0));
T32 = (DH(q3(t),0,0,sym(pi)/2)); 
T43 = (DH(q4(t),L3,0,-sym(pi)/2));
T54 = (DH(q5(t),0,0,sym(pi)/2));
T65 = (DH(q6(t),L4,0,0));
TE6 = DH(0,Le,0,0);
T20 = (T10*T21);
T30 = (T20*T32);
T40 = (T30*T43);
T50 = (T40*T54);
T60 = (T50*T65);
TE0 = simplify(T60*TE6);

% Create structure for transfromation matrix (useful to compute geometric Jacobian)
T = struct('T_Iminus1_0',{T00,T10,T20,T30,T40,T50},'JT',{'R','R','R','R','R','R'});
%% Robotics toolbox
g = [0 0 0 0 0 0];
L(1) = Link([g(1),L1,L0,(pi)/2],'standard'); 
L(2) = Link([g(2),0,L2,0],'standard');
L(3) = Link([g(3),0,0,pi/2],'standard');
L(4) = Link([g(4),L3,0,-(pi)/2],'standard');
L(5) = Link([g(5),0,0,(pi)/2],'standard');
L(6) = Link([g(6),L4,0,0],'standard');
tip = [0 0 Le]'; 
R = eye(3);
tool = [R tip;0 0 0 1]; 
IRB140 = SerialLink(L,'name','IRB140','tool',tool);

