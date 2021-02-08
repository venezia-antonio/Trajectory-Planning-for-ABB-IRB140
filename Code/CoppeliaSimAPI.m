%% This script allow the communication betweem MATLAB and CoppeliaSim
% Video recording reminder: 
% 2 displayed frames result in 1 recorded frame,100fps
% bullet 2.78, very accurate,dt = 200ms

robot = res;    % ABB IRB140 joint variables
plate = res_p;  % Plate joint variables
%% Connection to CoppeliaSim
vrep = remApi('remoteApi');
vrep.simxFinish(-1); 
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if(clientID > -1)  
    disp("Connection Opened");
    vrep.simxAddStatusbarMessage(clientID,'Connection from MATLAB',vrep.simx_opmode_oneshot);
    % Get Joint handle
    J_h = getJointHandle(vrep,clientID); 
    % Get bottle handle and initial position
    bottle_h = getBottleHandle(vrep,clientID);
    bottlePos = [-0.3454  -0.5003   0.3311]';
    % Get glass handle and position
    glass_h = getGlassHandle(vrep,clientID);
    glassPos = getGlassPosition(vrep,clientID,glass_h);
    % Set initial joint position
    setInitialJointPosition(vrep,clientID,robot,J_h)    
    % Start Simulation
    startSimulation(vrep,clientID)
    [~]=vrep.simxSynchronousTrigger(clientID);
  
    for i = 1:length(robot)  
        disp(i)
        [~]=vrep.simxSynchronousTrigger(clientID);
        tic
        % Get Joint angle
        jointAngle(i,1:8) = getJointPosition(vrep,clientID,J_h);
        % Get end effector position
        endEffectorPosition(:,i) = getEndEffectorPosition(vrep,clientID,J_h);
        % Dynamics control 
         setJointTargetPosition(vrep,clientID,robot(i,:),J_h);
         setJointTargetPosition_plate(vrep,clientID,plate(i,:),J_h)
        % Grasp the bottle
        if(abs(endEffectorPosition(:,i) - bottlePos) <= 1e-03)
            if(i<8000)
           closeGrip(vrep,clientID,bottle_h,J_h(7));
           disp('grip chiuso')
            end
        end
        if (abs(endEffectorPosition(:,i) - bottlePos) <= 1e-03)
            if(i>=8000)
           openGrip(vrep,clientID,bottle_h);
            end
        end
    end
openGrip(vrep,clientID,bottle_h)
pause(3)
[~] = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot); 
else
    disp("Fail");
end
%% Functions
function J_h = getJointHandle(vrep,clientID)
    [~,endEffector_h] = vrep.simxGetObjectHandle(clientID,'endEffector',vrep.simx_opmode_blocking);
    [~,J1_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint1',vrep.simx_opmode_blocking);
    [~,J2_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint2',vrep.simx_opmode_blocking);
    [~,J3_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint3',vrep.simx_opmode_blocking);
    [~,J4_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint4',vrep.simx_opmode_blocking);
    [~,J5_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint5',vrep.simx_opmode_blocking);
    [~,J6_h] = vrep.simxGetObjectHandle(clientID,'IRB140_joint6',vrep.simx_opmode_blocking);
    [~,J8_h] = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_blocking);
    [~,J9_h] = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_blocking);
    J_h = [J1_h,J2_h,J3_h,J4_h,J5_h,J6_h,endEffector_h,J8_h,J9_h];
end
function bottle_h = getBottleHandle(vrep,clientID)
    [~,bottle_h] = vrep.simxGetObjectHandle(clientID,'bottle',vrep.simx_opmode_blocking);
end
function glass_h = getGlassHandle(vrep,clientID)
    [~,glass_h] = vrep.simxGetObjectHandle(clientID,'glass',vrep.simx_opmode_blocking);
end
function bottlePosition = getBottlePosition(vrep,clientID,bottle_h)
    [~,bottlePosition] = vrep.simxGetObjectPosition(clientID,bottle_h,-1,vrep.simx_opmode_blocking);
end
function glassPosition = getGlassPosition(vrep,clientID,glass_h)
    [~,glassPosition] = vrep.simxGetObjectPosition(clientID,glass_h,-1,vrep.simx_opmode_blocking);
end
function [] = startSimulation(vrep,clientID)
    [~] = vrep.simxSynchronous(clientID,true);
    [~] = vrep.simxSynchronousTrigger(clientID);
    [~] = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
end
function [] = setInitialJointPosition(vrep,clientID,robot,J_h)
    [~] = vrep.simxSetJointPosition(clientID,J_h(1),robot(1,1),vrep.simx_opmode_blocking); %lag di 20ms circa costanti
    [~] = vrep.simxSetJointPosition(clientID,J_h(2),robot(1,2)-pi/2,vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(3),robot(1,3),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(4),robot(1,4),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(5),robot(1,5),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(6),robot(1,6),vrep.simx_opmode_blocking);
    [~] = vrep.simxSetJointPosition(clientID,J_h(8),deg2rad(0),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointPosition(clientID,J_h(9),deg2rad(0),vrep.simx_opmode_oneshot);
    pause(2)
end

function q = getJointPosition(vrep,clientID,J_h)
    [~,q1] = vrep.simxGetJointPosition(clientID,J_h(1),vrep.simx_opmode_blocking);
    [~,q2] = vrep.simxGetJointPosition(clientID,J_h(2),vrep.simx_opmode_blocking);
    [~,q3] = vrep.simxGetJointPosition(clientID,J_h(3),vrep.simx_opmode_blocking);
    [~,q4] = vrep.simxGetJointPosition(clientID,J_h(4),vrep.simx_opmode_blocking);
    [~,q5] = vrep.simxGetJointPosition(clientID,J_h(5),vrep.simx_opmode_blocking);
    [~,q6] = vrep.simxGetJointPosition(clientID,J_h(6),vrep.simx_opmode_blocking);
    [~,q7] = vrep.simxGetJointPosition(clientID,J_h(8),vrep.simx_opmode_blocking);
    [~,q8] = vrep.simxGetJointPosition(clientID,J_h(9),vrep.simx_opmode_blocking);
    q = [q1 q2 q3 q4 q5 q6 q7 q8];
end
function endEffector = getEndEffectorPosition(vrep,clientID,J_h)
    [~,endEffector] = vrep.simxGetObjectPosition(clientID,J_h(7),-1,vrep.simx_opmode_blocking);
end
function [] = setJointTargetPosition(vrep,clientID,robot,J_h)
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(1),robot(1,1),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(2),robot(1,2)-pi/2,vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(3),robot(1,3),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(4),robot(1,4),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(5),robot(1,5),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(6),robot(1,6),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(8),0,vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointTargetPosition(clientID,J_h(9),0,vrep.simx_opmode_oneshot);
end
function [] = setJointTargetPosition_plate(vrep,clientID,plate,J_h)
    [~] = vrep.simxSetJointPosition(clientID,J_h(8),plate(1,1),vrep.simx_opmode_oneshot);
    [~] = vrep.simxSetJointPosition(clientID,J_h(9),plate(1,2),vrep.simx_opmode_oneshot);
end
function [] = closeGrip(vrep,clientID,bottle_h,endEffector_h)  
    [~] = vrep.simxSetObjectParent(clientID,bottle_h,endEffector_h,true,false);
end
function [] = openGrip(vrep,clientID,bottle_h)  
    [~] = vrep.simxSetObjectParent(clientID,bottle_h,-1,true,false);
end

