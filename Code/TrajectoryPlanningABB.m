%% Main file 
clear all
close all
clc
% Add to path other folders
addpath('Checkpoints','Functions');

% In "createRobot.m" are defined transformation matrices of ABB IRB-140
% robot manipulator using Denavit-Hartenberg convention
run createRobot

% Step time
Ts = 0.2;

% %% Plot workspace
% % Step with which compute workspace
% angleStep = deg2rad(20);
% 
% % Angle limits
% th1 = (-pi:angleStep:pi);
% th2 = (-pi/2:angleStep:deg2rad(110));
% th3 = (deg2rad(-50):angleStep:deg2rad(230)) ;
% th4 = (deg2rad(-200):angleStep:deg2rad(200));
% th5 = (deg2rad(-120):angleStep:deg2rad(120));
% th6 = (deg2rad(-400):angleStep:deg2rad(400));
% q = {th1,th2,th3,th4,th5,th6};
% 
% % Function that allows to plot workspace
% plotWorkspace(TE0,q)

%% 1° trajectory (joint space)  -- from initial pose of manipulator
% Initial conditions
qi1 = [0 +pi/2 0 0 0 -pi/2];
dqi1 = zeros(1,6);
ddqi1 = zeros(1,6);
ti1 = 0;

% Final conditions
qf1 = [-1.8425    1.2918   -0.1196    1.4631    1.3208   -2.7295];
dqf1 = zeros(1,6);
ddqf1 = zeros(1,6);
tf1 = 2;

% Compute trajectory
[q1_1,q1_2,q1_3,q1_4,q1_5,q1_6,dq1_1,dq1_2,dq1_3,dq1_4,dq1_5,dq1_6,ddq1_1,ddq1_2,ddq1_3,ddq1_4,ddq1_5,ddq1_6] = trajPlan1(qi1,qf1,ti1,tf1,dqi1,dqf1,ddqi1,ddqf1,Ts);

% Joint variables for the plate
q1_7 = zeros(1,length(q1_1));
q1_8 = zeros(1,length(q1_1));

% Concatenation of vectors for joint variables of manipulator
res1 = [q1_1', q1_2', q1_3', q1_4', q1_5', q1_6'];
dres1 = [dq1_1;dq1_2;dq1_3;dq1_4;dq1_5;dq1_6]';
ddres1 = [ddq1_1;ddq1_2;ddq1_3;ddq1_4;ddq1_5;ddq1_6]';

% Compute end-effector linear velocity using geometric Jacobian
J1 = Jacobian(T,TE0,res1);
for i = 1:length(res1)
    J = J1(i).J;
    dp_s1(:,i) = J(1:3,:)*dres1(i,:)';
end

% Forward kinematics
for i = 1:length(res1)
tmp = fkine(TE0,res1(i,:));
p_s1(:,i) = tmp(1:3,end);
end

% Compute end-effector acceleration
ddp_s1 = EndEffAcc(TE0,res1,dres1,ddres1);

% Concatenation of vectors for joint variables of plate
res1_p = [q1_7', q1_8'];
% save('checkpoint1')
%% 2° trajectory (operational space) -- linear segment from top to bottom
Pin2 = [-0.3454  -0.5003   0.5506]';    % Initial end-effector position
Pf2 = [-0.3454  -0.5003   0.3306]';     % Final end-effector position
ti2 = tf1 ;                             % Initial time instant
tf2 = ti2 + 1;                          % Final time instant

% Compute trajectory
[p_s2,dp_s2,ddp_s2] = trajPlan2(Pin2,Pf2,ti2,tf2,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res1(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s2)
    R_t2(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res2 = solveIK(p_s2,R_t2,T30);

% Using inverse of Jacobian compute joint velocities
J2 = Jacobian(T,TE0,res2);
for i = 1:length(res2)
    J = inv(J2(i).J);
    dres2(i,:) = J(:,1:3)*dp_s2(:,i);
end

% Joint variables for the plate
q2_7 = zeros(length(p_s2),1);
q2_8 = zeros(length(p_s2),1);

% Concatenation of vectors for joint variables of plate
res2_p = [q2_7 q2_8];
% save('checkpoint2')
%% 3° trajectory (operational space) -- linear segment from bottom to top
Pin3 = [-0.3454  -0.5003   0.3306]';        % Initial end-effector position
Pf3 =  [-0.3454  -0.5003   0.5506]';        % Final end-effector position
ti3 = tf2;                                  % Initial time instant
tf3 = ti3 + 1;                              % Final time instant

% Compute trajectory
[p_s3,dp_s3,ddp_s3] = trajPlan2(Pin3,Pf3,ti3,tf3,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res2(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s3)
    R_t3(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res3 = solveIK(p_s3,R_t3,T30);

% Using inverse of Jacobian compute joint velocities
J3 = Jacobian(T,TE0,res3);
for i = 1:length(res3)
    J = inv(J3(i).J);
    dres3(i,:) = J(:,1:3)*dp_s3(:,i);
end

% Joint variables for the plate
q3_7 = linspace(0,-pi/2,length(res3));
q3_8 = zeros(1,length(res3));

% Concatenation of vectors for joint variables of plate
res3_p = [q3_7' q3_8'];
% save('checkpoint3')
%% 4° trajectory first half (operational space) -- from table to cork
Pcv = [-0.2794 -0.5013 0.2706]';        % Position of cork

Pin4_1 = Pf3;                           % Initial end-effector position
Pf4_1 =  Pcv + [0.2 0 -0.1155]';        % Final end-effector position 
ti4_1 = tf3 ;                           % Initial time instant
tf4_1 = ti4_1 + 1;                      % Final time instant

% Compute trajectory
[p_s4_1,dp_s4_1,ddp_s4_1] = trajPlan2(Pin4_1,Pf4_1,ti4_1,tf4_1,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res3(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s4_1)
    R_t4_1(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res4_1 = solveIK(p_s4_1,R_t4_1,T30);

% Using inverse of Jacobian compute joint velocities
J4_1 = Jacobian(T,TE0,res4_1);
for i = 1:length(res4_1)
    J = inv(J4_1(i).J);
    dres4_1(i,:) = J(:,1:3)*dp_s4_1(:,i);
end

%% 4° trajectory second half (operational space) -- from table to corkscrew
Pin4_2 = Pf4_1;                 % Initial end-effector position
Pf4_2 =  Pcv + [0 0 -0.1155]';  % Final end-effector position
ti4_2 = tf4_1;                  % Initial time instant
tf4_2 = ti4_2 + 1;              % Final time instant

% Compute trajectory
[p_s4_2,dp_s4_2,ddp_s4_2] = trajPlan2(Pin4_2,Pf4_2,ti4_2,tf4_2,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res3(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s4_2)
    R_t4_2(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res4_2 = solveIK(p_s4_2,R_t4_2,T30);

% Using inverse of Jacobian compute joint velocities
J4_2 = Jacobian(T,TE0,res4_2);
for i = 1:length(res4_2)
    J = inv(J4_2(i).J);
    dres4_2(i,:) = J(:,1:3)*dp_s4_2(:,i);
end

% Concatenation of vectors for joint variables of manipulator
res4 = [res4_1;res4_2];
dres4 = [dres4_1;dres4_2];
p_s4 = [p_s4_1 p_s4_2];
dp_s4 = [dp_s4_1 dp_s4_2];
ddp_s4 = [ddp_s4_1 ddp_s4_2];

% Joint variables of the plate
q4_7 = -pi/2*ones(1,length(res4));
q4_8 = zeros(1,length(res4));

% Concatenation of vectors for joint variables of plate
res4_p = [q4_7' q4_8'];
% save('checkpoint4')
%% 5° trajectory (operational space) -- Uncork
ti5 = tf4_2;                            % Initial time instant
tf5 = ti5 + 5;                          % Final time instant
radius = 0.1155;                        % Point belonging to versor r1
R5 = rotz(-pi/2)*roty(pi/2) ;           % Rotation matrix from end-effector frame to circumference frame
pt1 = p_s4_2(:,end);                    % Point belonging to circumference
r1 = R5*[0 0 1]';                       % Perpendicular axis versor to the plane of circumferenze
d1  = R5*[-radius 0 -1]' + pt1;         % Distance from end-effector frame to 'radius'
delta1 = pt1-d1;                        % Distance from 'pt1' to 'radius' on fixed frame
c1 = d1 + (delta1'*r1)*r1;              % Center of circumference    
ro1 = norm(pt1-c1);                     % Radius of circumference

% Compute curvilinear abscissa trajectory
[s5,ds5,dds5] = curv(0,pi*ro1/4,ti5,tf5,Ts);

% Compute trajectory of end-effector
[p_s5,dp_s5,ddp_s5] = trajCircle(r1,d1,pt1,s5,ds5,dds5,R5);

% During the uncork of the bottle, the orientation of end-effector must
% change: the end-effector axis coincident to the bottle one, must follows 
% the versor that defines the direction of the circumference's radius.
% The other two axes, must be chosen in order to obtain a left-handed
% frame.
% To do it, the function shakeBeer is used, considering alpha = 0.
alpha5 = 0;                                 % End-effector tilt angle                             
R5_1 = roty(pi/2);                      
T5 = shakeBeer(p_s5,c1,ro1,alpha5,R5_1);    % Compute transfromation matrices for the whole trajectory

% Solve inverse kinematics
res5 = solveIK(p_s5,T5,T30);

% Using inverse of Jacobian compute joint velocities
J5 = Jacobian(T,TE0,res5);
for i = 1:length(res5)
    J = inv(J5(i).J);
    dres5(i,:) = J(:,1:3)*dp_s5(:,i);
end

% Joint variables for the plate
q5_7 = -pi/2*ones(1,length(res5));
q5_8 = zeros(1,length(res5)); 

% Concatenation of vectors for joint variables of plate
res5_p = [q5_7' q5_8'];
% save checkpoint5
%% 6° trajectory (operational space) -- Leave the corkscrew
Pin6 = p_s5(:,end);                     % Initial end-effector position
Pf6 =  Pin6 + [0.06 0 0]';              % Finale end-effector position
ti6 = tf5;                              % Initial time instant
tf6 = ti6 + 2;                          % Final time instant

% Compute trajectory
[p_s6,dp_s6,ddp_s6] = trajPlan2(Pin6,Pf6,ti6,tf6,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res5(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s6)
    R_t6(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res6 = solveIK(p_s6,R_t6,T30);

% Using inverse of Jacobian compute joint velocities
J6 = Jacobian(T,TE0,res6);
for i = 1:length(res6)
    J = inv(J6(i).J);
    dres6(i,:) = J(:,1:3)*dp_s6(:,i);
end

% Joint variables for the plate
q6_7 = -pi/2*ones(1,length(res6));
q6_8 = zeros(1,length(res6));

% Concatenation of vectors for joint variables of plate
res6_p = [q6_7' q6_8'];
% save checkpoint6
%% 7° trajectory (joint space) -- Approch the glass
% Initial conditions
qi7 = res6(end,:);
dqi7 = zeros(1,6);
ddqi7 = zeros(1,6);
ti7 = tf6;

% Final conditions 
qf7 = [-2.1971    1.1262   -0.7534   -0.6605    1.2712    0.2254];
dqf7 = zeros(1,6);
ddqf7 = zeros(1,6);
tf7 = ti7 + 5;

% Compute trajectory
[q7_1,q7_2,q7_3,q7_4,q7_5,q7_6,dq7_1,dq7_2,dq7_3,dq7_4,dq7_5,dq7_6,ddq7_1,ddq7_2,ddq7_3,ddq7_4,ddq7_5,ddq7_6] = trajPlan1(qi7,qf7,ti7,tf7,dqi7,dqf7,ddqi7,ddqf7,Ts);
res7 = [q7_1', q7_2', q7_3', q7_4', q7_5', q7_6'];      % Joint positions
dres7 = [dq7_1;dq7_2;dq7_3;dq7_4;dq7_5;dq7_6]';         % Joint velocities
ddres7 = [ddq7_1;ddq7_2;ddq7_3;ddq7_4;ddq7_5;ddq7_6]';  % Joint acceleration

% Compute end-effector linear velocity using geometric Jacobian
J7 = Jacobian(T,TE0,res7);
for i = 1:length(res7)
    J = J7(i).J;
    dp_s7(:,i) = J(1:3,:)*dres7(i,:)';
end

% Forward kinematics
for i = 1:length(res7)
tmp = fkine(TE0,res7(i,:));
p_s7(:,i) = tmp(1:3,end);
end

% Compute end-effector linear acceleration 
ddp_s7 = EndEffAcc(TE0,res7,dres7,ddres7);

% Joint variables of the plate
q7_7 = -pi/2*ones(1,length(res7));

q7_8 = trajPlan1(0,pi/4,ti7,tf7,0,0,0,0,Ts);
% q7_8 = linspace(0,pi/4,length(res7));

% Concatenation of vectors for joint variables of plate
res7_p = [q7_7' q7_8'];
% save checkpoint7
%% 8° trajectory (operational space) -- Spill the beer (first part)
% Initial end-effector position
Pin8 = fkine(TE0(1:3,end),res7(end,:));
% Final end-effector position
Pf8 = [-0.3061,-0.5,0.3999]';
ti8 = tf7;                  % Initial time instant
tf8 = ti8 + 15;             % Final time instant

% Compute trajectory
[p_s8,dp_s8,ddp_s8] = trajPlan2(Pin8,Pf8,ti8,tf8,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res7(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s8)
    R_t8(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res8 = solveIK(p_s8,R_t8,T30);

% Using inverse of Jacobian compute joint velocities
J8 = Jacobian(T,TE0,res8);
for i = 1:length(res8)
    J = inv(J8(i).J);
    dres8(i,:) = J(:,1:3)*dp_s8(:,i);
end

% Joint variables for the plate computed using a quintic polynomial
q_iniz = [-pi/2 pi/4];
q_fin = [-pi/2 deg2rad(20)];
[q8_7,q8_8] = trajPlan1(q_iniz,q_fin,ti8,tf8,[0 0],[0 0],[0 0],[0 0],Ts);

% Concatenation of vectors for joint variables of plate
% res8_p = [q8_7p' q8_8p'];
res8_p = [q8_7' q8_8'];
% save checkpoint8
%% 9° trajectory (joint space) -- Approch shaking beer
ti9 = tf8;                  % Initial time instant
tf9 = ti9 + 2;              % Final time instant
qi9 = res8(end,:);          % Initial joint variables
qf9 = [-1.5708,1.0835,-0.5114,-0.0001,0.9986,-1.5707]; % Final joint variables
dqi9 = zeros(1,6);          % Initial joint velocities
ddqi9 = zeros(1,6);         % Initial joint accelerations
dqf9 = zeros(1,6);          % Final joint velocities
ddqf9 = zeros(1,6);         % Final joint velocities

% Compute trajectory
[q9_1,q9_2,q9_3,q9_4,q9_5,q9_6,dq9_1,dq9_2,dq9_3,dq9_4,dq9_5,dq9_6,ddq9_1,ddq9_2,ddq9_3,ddq9_4,ddq9_5,ddq9_6] = trajPlan1(qi9,qf9,ti9,tf9,dqi9,dqf9,ddqi9,ddqf9,Ts);
res9 = [q9_1',q9_2',q9_3',q9_4',q9_5',q9_6'];           % Joint variables position
dres9 = [dq9_1;dq9_2;dq9_3;dq9_4;dq9_5;dq9_6]';         % Joint variables velocity
ddres9 = [ddq9_1;ddq9_2;ddq9_3;ddq9_4;ddq9_5;ddq9_6]';  % Joint variables acceleration

% Compute end-effector linear velocity using geometric Jacobian
J9 = Jacobian(T,TE0,res9);
for i = 1:length(res9)
    J = J9(i).J;
    dp_s9(:,i) = J(1:3,:)*dres9(i,:)';
end

% Forward kinematics
for i = 1:length(res9)
tmp = fkine(TE0,res9(i,:));
p_s9(:,i) = tmp(1:3,end);
end

% Compute end-effector acceleration
ddp_s9 = EndEffAcc(TE0,res9,dres9,ddres9);

% Joint variables for the plate
q9_7 = q8_7(end)*ones(1,length(res9));
q9_8 = q8_8(end)*ones(1,length(res9));

% Concatenation of vectors for joint variables of plate
res9_p = [q9_7' q9_8'];
% save('checkpoint9')
%%  10° trajectory (operational space) -- Circular path
n = 4;                          % Number of circular path
ti10 = tf9;                     % Initial time instant
tf10 = ti10 + 3*n;              % Final time instant
pt = p_s9(:,end);               % Point belonging to circumference
r = [0 0 1]';                   % Perpendicular axis versor to the plane of circumferenze
R10 = rotz(-pi);                % Rotation matrix from end-effector frame to circumference frame
d = [pt(1)+0.02 pt(2) pt(3)+1]';    % Distance from end-effector frame to 'radius'        
delta = pt-d;                   % Distance from 'pt1' to 'radius' on fixed frame
c = d + (delta'*r)*r;           % Center of circumference
ro = norm(pt-c);                % radius of circumference

% Compute curvilinear abscissa trajectory
[s10,ds10,dds10] = curv(0,n*2*pi*ro,ti10,tf10,Ts);

% Compute trajectory of end-effector
[p_s10,dp_s10,ddp_s10] = trajCircle(r,d,pt,s10,ds10,dds10,R10);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation 
tmp = fkine(TE0,res9(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s10)
    R_t10(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res10 = solveIK(p_s10,R_t10,T30);

% Using inverse of Jacobian compute joint velocities
J10 = Jacobian(T,TE0,res10);
for i = 1:length(res10)
    J = inv(J10(i).J);
    dres10(i,:) = J(:,1:3)*dp_s10(:,i);
end

% Joint variables for the plate
q10_7 = q8_7(end)*ones(1,length(res10));
q10_8 = q8_8(end)*ones(1,length(res10));

% Concatenation of vectors for joint variables of plate
res10_p = [q10_7' q10_8'];
% save('checkpoint10')
%% 11° trajectory (operational space) -- Shake
ti11 = tf10;                        % Initial time instant
tf11 = ti11 + 3*n;                  % Final time instant
alpha11 = sym(5*pi/12);             % End-effector tilt angle
R11 = roty(pi);

% Compute rotation matrix for the whole trajectory 
Tr = shakeBeer(p_s10,c,ro,alpha11,R11);

% Solve inverse kinematics
res11 = solveIK(p_s10,Tr,T30);
p_s11 = p_s10;                  % Position of end-effector
dp_s11 = dp_s10;                % Velocity of end-effector
ddp_s11 = ddp_s10;              % Acceleration of end-effector

% Using inverse of Jacobian compute joint velocities
J11 = Jacobian(T,TE0,res11);
for i = 1:length(res11)
    J = inv(J11(i).J);
    dres11(i,:) = J(:,1:3)*dp_s11(:,i);
end

% Joint variables for the plate
q11_7 = q8_7(end)*ones(1,length(res11));
q11_8 = q8_8(end)*ones(1,length(res11));

% Concatenation of vectors for joint variables of plate
res11_p = [q11_7' q11_8'];
% save('checkpoint11')
%% 11° trajectory (joint space) -- Changing orientation of end-effector from 10° to 11° trajectory
% Initial conditions
qi10_1 = res10(end,:);
dqi10_1 = zeros(1,6);
ddqi10_1 = zeros(1,6);
ti10_1 = tf10;

% Final conditions 
qf10_1 = res11(1,:);
dqf10_1 = zeros(1,6);
ddqf10_1 = zeros(1,6);
tf10_1 = ti10_1 + 1;

% Compute trajectory
[q10_1_1,q10_1_2,q10_1_3,q10_1_4,q10_1_5,q10_1_6,dq10_1_1,dq10_1_2,dq10_1_3,dq10_1_4,dq10_1_5,dq10_1_6,ddq10_1_1,ddq10_1_2,ddq10_1_3,ddq10_1_4,ddq10_1_5,ddq10_1_6] = trajPlan1(qi10_1,qf10_1,ti10_1,tf10_1,dqi10_1,dqf10_1,ddqi10_1,ddqf10_1,Ts);

% Concatenation of vectors for joint variables of manipulator
res10_1 = [q10_1_1', q10_1_2', q10_1_3', q10_1_4', q10_1_5', q10_1_6'];
dres10_1 = [dq10_1_1;dq10_1_2;dq10_1_3;dq10_1_4;dq10_1_5;dq10_1_6]';
ddres10_1 = [ddq10_1_1;ddq10_1_2;ddq10_1_3;ddq10_1_4;ddq10_1_5;ddq10_1_6]';

% Compute end-effector linear velocity using geometric Jacobian
J10_1 = Jacobian(T,TE0,res10_1);
for i = 1:length(res10_1)
    J = J10_1(i).J;
    dp_s10_1(:,i) = J(1:3,:)*dres10_1(i,:)';
end

% Forward kinematics
for i = 1:length(res10_1)
tmp = fkine(TE0,res10_1(i,:));
p_s10_1(:,i) = tmp(1:3,end);
end

% Compute end-effector linear acceleration
ddp_s10_1 = EndEffAcc(TE0,res10_1,dres10_1,ddres10_1);

% Joint variables for the plate
q10_1_7 = q8_7(end)*ones(1,length(res10_1));
q10_1_8 = q8_8(end)*ones(1,length(res10_1));

% Concatenation of vectors for joint variables of plate
res10_1_p = [q10_1_7' q10_1_8'];
% save checkpoint10_1
%% 12° trajectory (joint space) --  Coming back to glass
ti12 = tf11;                        % Initial time instant
tf12 = ti12 + 1;                    % Final time instant

% Flip results of 9° trajectory (the initial and final point are the inverse of 9° trajectory)
res12 = flip(res9);
dres12 = flip(dres9);
ddres12 = flip(ddres9);
p_s12 = flip(p_s9,2);
dp_s12 = flip(dp_s9,2);
ddp_s12 = flip(ddp_s9,2);

% Joint variables for the plate
q12_7 = q8_7(end)*ones(1,length(res12));
q12_8 = q8_8(end)*ones(1,length(res12));

% Concatenation of vectors for joint variables of plate
res12_p = [q12_7' q12_8'];
% save('checkpoint12')
%% 11° trajectory (joint space) -- Changing orientation of end-effector from 11° to 12° trajectory
% Initial conditions
qi11_1 = res11(end,:);
dqi11_1 = zeros(1,6);
ddqi11_1 = zeros(1,6);
ti11_1 = tf11;

% Final conditions 
qf11_1 = res12(1,:);
dqf11_1 = zeros(1,6);
ddqf11_1 = zeros(1,6);
tf11_1 = ti11_1 + 1;

% Compute trajectory
[q11_1_1,q11_1_2,q11_1_3,q11_1_4,q11_1_5,q11_1_6,dq11_1_1,dq11_1_2,dq11_1_3,dq11_1_4,dq11_1_5,dq11_1_6,ddq11_1_1,ddq11_1_2,ddq11_1_3,ddq11_1_4,ddq11_1_5,ddq11_1_6] = trajPlan1(qi11_1,qf11_1,ti11_1,tf11_1,dqi11_1,dqf11_1,ddqi11_1,ddqf11_1,Ts);

% Concatenation of vectors for joint variables of manipulator
res11_1 = [q11_1_1', q11_1_2', q11_1_3', q11_1_4', q11_1_5', q11_1_6'];
dres11_1 = [dq11_1_1;dq11_1_2;dq11_1_3;dq11_1_4;dq11_1_5;dq11_1_6]';
ddres11_1 = [ddq11_1_1;ddq11_1_2;ddq11_1_3;ddq11_1_4;ddq11_1_5;ddq11_1_6]';

% Compute end-effector linear velocity using geometric Jacobian
J11_1 = Jacobian(T,TE0,res11_1);
for i = 1:length(res11_1)
    J = J11_1(i).J;
    dp_s11_1(:,i) = J(1:3,:)*dres11_1(i,:)';
end

% Forward kinematics
for i = 1:length(res11_1)
tmp = fkine(TE0,res11_1(i,:));
p_s11_1(:,i) = tmp(1:3,end);
end

% Compute end-effector linear acceleration
ddp_s11_1 = EndEffAcc(TE0,res11_1,dres11_1,ddres11_1);

% Joint variables for the plate
q11_1_7 = q8_7(end)*ones(1,length(res11_1));
q11_1_8 = q8_8(end)*ones(1,length(res11_1));

% Concatenation of vectors for joint variables of plate
res11_1_p = [q11_1_7' q11_1_8'];
% save checkpoint11_1
%% 13° trajectory (operational space) -- Spill the beer (second part) by tipping the bottle
% Initial end-effector position
Pin13 = fkine(TE0(1:3,end),res8(end,:));
% Final end-effector position
Pf13 = [-0.4   -0.5    0.4772]';
ti13 = tf12;                % Initial time instant
tf13 = ti13 + 1;            % Final time instant

% Compute trajectory
[p_s13,dp_s13,ddp_s13] = trajPlan2(Pin13,Pf13,ti13,tf13,Ts);

% Initial rotation matrix
Rin = fkine(TE0(1:3,1:3),res12(end,:));

% Axis along which must be rotate the end-effector
axis = [0 1 0]';

% Angle generation
% Initial conditions
anglei = [0 0 0 0 0 0];
danglei = [0 0 0 0 0 0];
ddanglei = [0 0 0 0 0 0];
ti_angle = tf12;

% Final conditions 
anglef = [-pi/6 1 1 1 1 1];
danglef = [0 0 0 0 0 0];
ddanglef = [0 0 0 0 0 0];
tf_angle = ti_angle + 1;

% Compute orientation trajectory
[angle,~,~,~,~,~] = trajPlan1(anglei,anglef,ti_angle,tf_angle,danglei,danglef,ddanglei,ddanglef,Ts);

% Initialization of structure containing rotation matrices using axis-angle
% representation
for i = 1:length(p_s13)
    R_t13(i).R = axisAngle(axis,angle(i))*Rin;   
end

% Solve inverse kinematics
res13 = solveIK(p_s13,R_t13,T30);

% Using inverse of Jacobian compute joint velocities
J13 = Jacobian(T,TE0,res13);
for i = 1:length(res13)
    J = inv(J13(i).J);
    dres13(i,:) = J(:,1:3)*dp_s13(:,i);
end

% Joint variables for the plate using quintic polynomial
q_iniz13 = q_fin;
q_fin13 = [-pi/2 0];
[q13_7,q13_8] = trajPlan1(q_iniz13,q_fin13,ti13,tf13,[0 0],[0 0],[0 0],[0 0],Ts);

% Concatenation of vectors for joint variables of plate
res13_p = [q13_7' q13_8'];
% save checkpoint13
%% 14° trajectory (operational space) -- Drain the beer 30%
Pin14_1 = p_s13(:,end);                 % Initial end-effector position
Pf14_1 =  Pin14_1 + [0 0 0.3*0.35]';    % Final end-effector position
ti14_1 = tf13;                          % Initial time instant
tf14_1 = ti14_1 + 1;                    % Final time instant

% Compute trajectory 
[p_s14_1,dp_s14_1,ddp_s14_1] = trajPlan2(Pin14_1,Pf14_1,ti14_1,tf14_1,Ts);

% Concatenations of vectors
p_s14_1 = [p_s14_1,flip(p_s14_1,2),p_s14_1,flip(p_s14_1,2)];
dp_s14_1 = [dp_s14_1,flip(dp_s14_1,2),dp_s14_1,flip(dp_s14_1,2)];
ddp_s14_1 = [ddp_s14_1,flip(ddp_s14_1,2),ddp_s14_1,flip(ddp_s14_1,2)];

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation 
tmp = fkine(TE0,res13(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s14_1)
    R_t14_1(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res14_1 = solveIK(p_s14_1,R_t14_1,T30);

% Using inverse of Jacobian compute joint velocities
J14_1 = Jacobian(T,TE0,res14_1);
for i = 1:length(res14_1)
    J = inv(J14_1(i).J);
    dres14_1(i,:) = J(:,1:3)*dp_s14_1(:,i);
end

% Joint variables for the plate
q14_7_1 = -pi/2*ones(1,length(res14_1));
q14_8_1 = zeros(1,length(res14_1));

% Concatenation of vectors for joint variables of plate
res14_p_1 = [q14_7_1' q14_8_1'];
% save('checkpoint14')
%% 14° trajectory (operational space) -- Drain the beer 100%
Pin14_2 = p_s13(:,end);             % Initial end-effector position
Pf14_2 =  Pin14_2 + [0 0 0.35]';    % Final end-effector position    
ti14_2 = tf13;                      % Initial time instant
tf14_2 = ti14_2 + 1;                % Final time instant

% Compute trajectory 
[p_s14_2,dp_s14_2,ddp_s14_2] = trajPlan2(Pin14_2,Pf14_2,ti14_2,tf14_2,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation 
tmp = fkine(TE0,res13(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s14_2)
    R_t14_2(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res14_2 = solveIK(p_s14_2,R_t14_2,T30);

% Using inverse of Jacobian compute joint velocities
J14_2 = Jacobian(T,TE0,res14_2);
for i = 1:length(res14_2)
    J = inv(J14_2(i).J);
    dres14_2(i,:) = J(:,1:3)*dp_s14_2(:,i);
end

% Joint variables for the plate
q14_7_2 = -pi/2*ones(1,length(res14_2));
q14_8_2 = zeros(1,length(res14_2));

% Concatenation of vectors for joint variables of plate
res14_p_2 = [q14_7_2' q14_8_2'];
% save('checkpoint14')
%% 15° trajectory (operational space) -- Flip the bottle 
ti15 = tf14_2;                      % Initial time instant
tf15 = ti15 + 4;                    % Final time instant
radius = 0.1155;                    % Point belonging to versor r1
R15 = rotx(-pi/2)*rotz(-pi/6) ;     % Rotation matrice from circumference frame to end-effector frame
pt15 = p_s14_2(:,end);              % Point belonging to circumference
r15 = R15*[0 0 1]';                 % Perpendicular axis versor to the plane of circumferenze
d15  = R15*[-radius 0 -1]' + pt15;  % Distance from end-effector frame to 'radius'
delta15 = pt15-d15;                 % Distance from 'pt1' to 'radius' on fixed frame
c15 = d15 + (delta15'*r15)*r15;     % Center of circumference
ro15 = norm(pt15-c15);              % Radius of circumference

% Compute curvilinear abscissa trajectory
[s15,ds15,dds15] = curv(0,(pi/2 + pi/6)*ro15,ti15,tf15,Ts);

% Compute trajectory of end-effector
[p_s15,dp_s15,ddp_s15] = trajCircle(r15,d15,pt15,s15,ds15,dds15,R15);
alpha15 = 0;                            % End-effector tilt angle
R15_1 = roty(-pi/2)*roty(-pi/2);

% Compute transfromation matrices for the whole trajectory
T15 = shakeBeer(p_s15,c15,ro15,alpha15,R15_1); 

% Solve inverse kinematics
res15 = solveIK(p_s15,T15,T30);

% Using inverse of Jacobian compute joint velocities
J15 = Jacobian(T,TE0,res15);
for i = 1:length(res15)
    J = inv(J15(i).J);
    dres15(i,:) = J(:,1:3)*dp_s15(:,i);
end

% Joint variables for the plate
q15_7 = -pi/2*ones(1,length(res15));
q15_8 = zeros(1,length(res15));

% Concatenation of vectors for joint variables of plate
res15_p = [q15_7' q15_8'];
% save checkpoint15
%% 16° trajectory (operational space) -- Rest the bottle
% Initial end-effector position
Pin16 = fkine(TE0(1:3,end),res15(end,:));
% Final end-effector position
Pf16 = Pf2;
ti16 = tf15;        % Initial time instant
tf16 = ti16 + 3;    % Final time instant 

% Compute trajectory 
[p_s16,dp_s16,ddp_s16] = trajPlan2(Pin16,Pf16,ti16,tf16,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res15(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s16)
    R_t16(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res16 = solveIK(p_s16,R_t16,T30);

% Using inverse of Jacobian compute joint velocities
J16 = Jacobian(T,TE0,res16);
for i = 1:length(res16)
    J = inv(J16(i).J);
    dres16(i,:) = J(:,1:3)*dp_s16(:,i);
end

% Joint variables for the plate
q16_7 = -pi/2*ones(1,length(res16));
q16_8 = zeros(1,length(res16));

% Concatenation of vectors for joint variables of plate
res16_p = [q16_7' q16_8'];
% save checkpoint16
%% 17° trajectory (operational space) -- Leave the bottle on the table
Pin17 = [-0.3454  -0.5003   0.3306]';       % Initial end-effector position
Pf17 =  [-0.3454  -0.5003   0.5506]';       % Final end-effector position    
ti17 = tf16;                                % Initial time instant
tf17 = ti17 + 2;                            % Final time instant

% Compute trajectory 
[p_s17,dp_s17,ddp_s17] = trajPlan2(Pin17,Pf17,ti17,tf17,Ts);

% Compute TE0 matrix for the whole trajectory in order to take the
% orientation
tmp = fkine(TE0,res16(end,:));

% Initialization of structure containing rotation matrices
for i = 1:length(p_s17)
    R_t17(i).R = tmp(1:3,1:3);   
end

% Solve inverse kinematics
res17 = solveIK(p_s17,R_t17,T30);

% Using inverse of Jacobian compute joint velocities
J17 = Jacobian(T,TE0,res17);
for i = 1:length(res17)
    J = inv(J17(i).J);
    dres17(i,:) = J(:,1:3)*dp_s17(:,i);
end

% Joint variables for the plate
q17_7 = -pi/2*ones(1,length(res17));
q17_8 = zeros(1,length(res17));

% Concatenation of vectors for joint variables of plate
res17_p = [q17_7' q17_8'];
% save('checkpoint17')
%% 18° trajectory (joint space) --  Return the manipulator to initial configuration 
% Initial conditions
qi18 = res17(end,:);
dqi18 = zeros(1,6);
ddqi18 = zeros(1,6);
ti18 = tf17;

% Final conditions 
qf18 = [0 +pi/2 0 0 0 -pi/2];
dqf18 = zeros(1,6);
ddqf18 = zeros(1,6);
tf18 = ti18 + 2;

% Compute trajectory 
[q18_1,q18_2,q18_3,q18_4,q18_5,q18_6,dq18_1,dq18_2,dq18_3,dq18_4,dq18_5,dq18_6,ddq18_1,ddq18_2,ddq18_3,ddq18_4,ddq18_5,ddq18_6] = trajPlan1(qi18,qf18,ti18,tf18,dqi18,dqf18,ddqi18,ddqf18,Ts);
res18 = [q18_1', q18_2', q18_3', q18_4', q18_5', q18_6'];
dres18 = [dq18_1;dq18_2;dq18_3;dq18_4;dq18_5;dq18_6]';
ddres18 = [ddq18_1;ddq18_2;ddq18_3;ddq18_4;ddq18_5;ddq18_6]';

% Compute end-effector linear velocity using geometric Jacobian
J18 = Jacobian(T,TE0,res18);
for i = 1:length(res18)
    J = J18(i).J;
    dp_s18(:,i) = J(1:3,:)*dres18(i,:)';
end

% Forward kinematics
for i = 1:length(res18)
tmp = fkine(TE0,res18(i,:));
p_s18(:,i) = tmp(1:3,end);
end

% Compute end-effector linear acceleration 
ddp_s18 = EndEffAcc(TE0,res18,dres18,ddres18);

% Joint variables for the plate
q18_7 = -pi/2*ones(1,length(res18));
q18_8 = zeros(1,length(q18_1));

% Concatenation of vectors for joint variables of plate
res18_p = [q18_7', q18_8'];
% save checkpoint18
%% Concatentation of all trajectories
% Joint variables
res = [res1;res2;res3;res4;res5;res6;res7;res8;res9;res10;res10_1;res11;res11_1;res12;res13;res14_1;res14_2;res15;res16;res17;res18];
res_p = [res1_p;res2_p;res3_p;res4_p;res5_p;res6_p;res7_p;res8_p;res9_p;res10_p;res10_1_p;res11_p;res11_1_p;res12_p;res13_p;res14_p_1;res14_p_2;res15_p;res16_p;res17_p;res18_p];
dres = [dres1;dres2;dres3;dres4;dres5;dres6;dres7;dres8;dres9;dres10;dres10_1;dres11;dres11_1;dres12;dres13;dres14_1;dres14_2;dres15;dres16;dres17;dres18];

% End-Effector
p = [p_s1 p_s2 p_s3 p_s4 p_s5 p_s6 p_s7 p_s8 p_s9 p_s10 p_s10_1 p_s11 p_s11_1 p_s12 p_s13 p_s14_1 p_s14_2 p_s15 p_s16 p_s17 p_s18];
dp = [dp_s1 dp_s2 dp_s3 dp_s4 dp_s5 dp_s6 dp_s7 dp_s8 dp_s9 dp_s10 dp_s10_1 dp_s11 dp_s11_1 dp_s12 dp_s13 dp_s14_1 dp_s14_2 dp_s15 dp_s16 dp_s17 dp_s18];
ddp = [ddp_s1 ddp_s2 ddp_s3 ddp_s4 ddp_s5 ddp_s6 ddp_s7 ddp_s8 ddp_s9 ddp_s10 ddp_s10_1 dp_s11 ddp_s11_1 ddp_s12 ddp_s13 ddp_s14_1 ddp_s14_2 ddp_s15 ddp_s16 ddp_s17 ddp_s18];
% save checkpoint19
%% Plot Joint variables
t = (0:length(res)-1)*Ts;
y = [-2*(pi):(pi/4):2*(pi)];
yC = {'-2\pi', '-7\pi/4', '-3\pi/2', '-5\pi/4', '-\pi', '-3\pi/4', '-\pi/2', '-\pi/4', '0', '\pi/4', '\pi/2', '3\pi/4', '\pi', '5\pi/4', '3\pi/2', '7\pi/4', '2\pi'};

% Joint positions
figure()
hold on
sgtitle('Angular Position')
subplot(321)
hold on,grid minor
title('$${\theta}_{1} [rad]$$','interpreter','latex')
plot(t,res(:,1));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(322)
hold on,grid minor
title('$${\theta}_{2} [rad]$$','interpreter','latex')
plot(t,res(:,2));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(323)
hold on,grid minor
title('$${\theta}_{3} [rad]$$','interpreter','latex')
hold on
plot(t,res(:,3));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(324)
hold on,grid minor
title('$${\theta}_{4} [rad]$$','interpreter','latex')
hold on
plot(t,res(:,4));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(325)
hold on,grid minor
title('$${\theta}_{5} [rad]$$','interpreter','latex')
plot(t,res(:,5));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(326)
hold on,grid minor
title('$${\theta}_{6} [rad]$$','interpreter','latex')
plot(t,res(:,6));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels

% Joint velocities
figure()
sgtitle('Angular Velocity')
subplot(321)
hold on,grid minor
title('$$\dot{\theta}_{1} [rad/s]$$','interpreter','latex')
plot(t,dres(:,1));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(322)
hold on,grid minor
title('$$\dot{\theta}_{2} [rad/s]$$','interpreter','latex')
plot(t,dres(:,2));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(323)
hold on,grid minor
title('$$\dot{\theta}_{3} [rad/s]$$','interpreter','latex')
hold on
plot(t,dres(:,3));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(324)
hold on,grid minor
title('$$\dot{\theta}_{4} [rad/s]$$','interpreter','latex')
hold on
plot(t,dres(:,4));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(325)
hold on,grid minor
title('$$\dot{\theta}_{5} [rad/s]$$','interpreter','latex')
plot(t,dres(:,5));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels
subplot(326)
hold on,grid minor
title('$$\dot{\theta}_{6} [rad/s]$$','interpreter','latex')
plot(t,dres(:,6));xlabel('Time [s]');
set(gca,'ytick',y) % where to set the tick marks
set(gca,'yticklabels',yC) % give them user-defined labels


%% Plot trajectory in the operational space using PeterCorke Toolbox
%Plot 3d Animation
figure()
hold on,grid minor
view(3)
plot3(p(1,:),p(2,:),p(3,:),'black','LineWidth',2);
xlim([-1 1.5]);ylim([-1 1]);zlim([0 1.5]);
hold on
IRB140.plot(res(end,:));
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')

%% Plot End-Effector position, velocity and acceleration
figure()
sgtitle('End Effector position, velocity and acceleration')
subplot(331)
plot(t,p(1,:))
xlabel('Time [s]'),ylabel('p_x [m]'),grid minor
subplot(334)
plot(t,p(2,:))
xlabel('Time [s]'),ylabel('p_y [m]'),grid minor
subplot(337)
plot(t,p(3,:))
xlabel('Time [s]'),ylabel('p_z [m]'),grid minor
subplot(332)
plot(t,dp(1,:))
xlabel('Time [s]'),ylabel('v_x [m/s]'),grid minor
subplot(335)
plot(t,dp(2,:))
xlabel('Time [s]'),ylabel('v_y [m/s]'),grid minor
subplot(338)
plot(t,dp(3,:))
xlabel('Time [s]'),ylabel('v_z [m/s]'),grid minor
subplot(333)
plot(t,ddp(1,:))
xlabel('Time [s]'),ylabel('a_x [m/s^2]'),grid minor
subplot(336)
plot(t,ddp(2,:))
xlabel('Time [s]'),ylabel('a_y [m/s^2]'),grid minor
subplot(339)
plot(t,ddp(3,:))
xlabel('Time [s]'),ylabel('a_z [m/s^2]'),grid minor

% Run coppeliaSimAPI to start communication with Coppelia's scenario
% run CoppeliaSimAPI
