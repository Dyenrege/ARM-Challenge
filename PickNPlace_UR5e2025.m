%% Pick and Place Starter Code
%
% This script shows how to use the motion control utilities to command the
% robot to pickup a bottle and drop it off at bin by using a sequential
% approach

% Execute each section individually and visualize result in Gazebo

%% Setup
rosIP = "192.168.2.130" % Change IP address!!    % IP address of ROS enabled machine  
rosshutdown; % shut down existing connection to ROS
rosinit(rosIP,11311);
% rosinit(rosIP,'NodeHost','192.168.206.1')
tftree = rostf;
pause(2);

[gripAct,gripGoal] = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct')
gripAct.FeedbackFcn = [];
gripAct.ResultFcn = [];

gripPos = 0.115; %0 is fully closed, 4 is fully open
gripGoal=packGripGoal(gripPos,gripGoal)
sendGoal(gripAct,gripGoal);

[trajAct,trajGoal] = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct') 
trajAct.FeedbackFcn = []; 
trajAct.ResultFcn = []; 
jointSub = rossubscriber("/joint_states",'DataFormat','struct')
jointStateMsg = jointSub.LatestMessage

UR5e = loadrobot('universalUR5e')
initialIKGuess = homeConfiguration(UR5e);

% Adjust body transformations from previous URDF version
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));
tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));
tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation

%% Set Init Configuration
% A good initial condition will help the inverse kinematics algorithm
% maintain the elbow away from the table
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
eeTransform = getTransform(tftree, 'base_link', 'tool0', rostime('now'));
trans = eeTransform.Transform.Translation;
currentXYZ = [trans.X; trans.Y; trans.Z]; %gets current end effector coordinates
disp('--- Current Tool0 Position ---');
disp(['Y = ', num2str(currentXYZ(1)), ' m']);
disp(['X = ', num2str(currentXYZ(2)), ' m']);
disp(['Z = ', num2str(currentXYZ(3)), ' m']);

%% Send new configuration (Align with bottle)
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.46 -0.07 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
eeTransform = getTransform(tftree, 'base_link', 'tool0', rostime('now'));
trans = eeTransform.Transform.Translation;
currentXYZ = [trans.X; trans.Y; trans.Z];
disp('--- Current Tool0 Position ---');
disp(['Y = ', num2str(currentXYZ(1)), ' m']);
disp(['X = ', num2str(currentXYZ(2)), ' m']);
disp(['Z = ', num2str(currentXYZ(3)), ' m']);

%% Approach bottle and close gripper
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.462 -0.07 0.242]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%% Close gripper
gripGoal=packGripGoal(0.515,gripGoal)
sendGoal(gripAct,gripGoal);

%% Lift bottle 
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.07 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 

%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 

%% Open gripper
gripGoal=packGripGoal(0.1,gripGoal)
sendGoal(gripAct,gripGoal);
%% purple c
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.695 -0.08 0.137]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%% close purple
gripGoal=packGripGoal(0.59,gripGoal)
sendGoal(gripAct,gripGoal);
%% lift purple c
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.7 -0.08 0.2]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%% green bin-purple
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
eeTransform = getTransform(tftree, 'base_link', 'tool0', rostime('now'));
trans = eeTransform.Transform.Translation;
currentXYZ = [trans.X; trans.Y; trans.Z];
disp('--- Current Tool0 Position ---');
disp(['Y = ', num2str(currentXYZ(1)), ' m']);
disp(['X = ', num2str(currentXYZ(2)), ' m']);
disp(['Z = ', num2str(currentXYZ(3)), ' m']);
%% place in green bin -purple
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%% Open gripper
gripGoal=packGripGoal(0.1,gripGoal)
sendGoal(gripAct,gripGoal);
%% go to blue cube
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%% apr blue cube
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.69 -0.08 0.112]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%% close gripper b cube
gripGoal=packGripGoal(0.56,gripGoal)
sendGoal(gripAct,gripGoal);
%% Scale coord
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [-0.35 0.58 0.6]; gripperRotation = [-pi/2 pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%% Scale coord move 2 nah
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [-0.35 0.58 0.55]; gripperRotation = [-pi/6 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.65 0.022 0.35]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.66 0.022 0.25]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.366 -0.01 0.35]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.366 -0.01 0.235]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.366 -0.01 0.35]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%% AICI
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.535 -0.07 0.15]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.535 0.09 0.07]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0.515,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0.0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.66 0.021 0.28]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.66 0.021 0.125]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.66 0.021 0.28]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.65 -0.001 0.1]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.63 -0.001 0.07]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.666,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.65 -0.088 0.2]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.695 -0.0823 0.138]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.630,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0.3;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.404 0.6 0.1277]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
gripGoal=packGripGoal(0.660,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.65 -0.088 0.2]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.695 -0.0823 0.116]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.650,gripGoal)
sendGoal(gripAct,gripGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.5 0.27 0.4]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.5 0.27 0.13]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.227,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.47 0.38 0.2]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.47 0.38 0.1]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.325,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal); 
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.62 0.28 0.2]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.62 0.28 0.08]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%% Open gripper
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0.3;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.235 0.17 0.35]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.235 0.17 0.25]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% Close gripper
gripGoal=packGripGoal(0.515,gripGoal)
sendGoal(gripAct,gripGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.235 0.17 0.35]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0.3;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%% 
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0.3;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.15 0.405 0.3]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.15 0.405 0.24]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% Close gripper
gripGoal=packGripGoal(0.515,gripGoal)
sendGoal(gripAct,gripGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.15 0.405 0.3]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0.3;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal);
%% 
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.1;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.22 0.34 0.2]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.22 0.33 0.07]; gripperRotation = [-pi -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% Close gripper
gripGoal=packGripGoal(0.205,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%% Close gripper
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.02 0.329 0.2]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.02 0.329 0.08]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% Close gripper
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.02 0.38 0.15]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.02 0.38 0.065]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.666,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.16 0.78 0.3]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [0.16 0.78 0.24]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%%
gripGoal=packGripGoal(0.515,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal); 
%%
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% Sticla rosie
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.03 0.91 0.2]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% Sticla rosie
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.03 0.91 0.08]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% Sticla rosie
gripGoal=packGripGoal(0.209,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% Place bottle above bin
jointStateMsg = receive(jointSub,5) % receive current robot configuration

gripperTranslation = [0.46 -0.3 0.3]; gripperRotation = [pi/2 -pi 0]; %  [Z Y X]radian

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal); sendGoal(trajAct,trajGoal);
%% Sticla rosie
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = -1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.03 0.797 0.2]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.03 0.797 0.13]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal); 
%% 
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.165 0.702 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.169 0.702 0.13]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
gripGoal=packGripGoal(0.225,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 1.5;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.2 0.8 0.2]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.2 0.8 0.09]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
gripGoal=packGripGoal(0.250,gripGoal)
sendGoal(gripAct,gripGoal);
%%
initialIKGuess(3).JointPosition = 0.3; initialIKGuess(1).JointPosition = 0;
trajGoal = packTrajGoal(initialIKGuess,trajGoal)
sendGoal(trajAct,trajGoal);
%%
jointStateMsg = receive(jointSub,5) % receive current robot configuration
show(UR5e,packIKGuess(initialIKGuess,jointStateMsg))

gripperTranslation = [-0.48 -0.34 0.25]; gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

tform = eul2tform(gripperRotation); tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] =ik('tool0',tform,ikWeights,initialIKGuess);
trajGoal = packTrajGoal(configSoln,trajGoal);
sendGoal(trajAct,trajGoal);
%% 
gripGoal=packGripGoal(0,gripGoal)
sendGoal(gripAct,gripGoal);
%% Helper functions
function initialIKGuess = packIKGuess(initialIKGuess,jointStateMsg)
    initialIKGuess(1).JointPosition = jointStateMsg.Position(4); % update configuration in initial guess
    initialIKGuess(2).JointPosition = jointStateMsg.Position(3);
    initialIKGuess(3).JointPosition = jointStateMsg.Position(1);
    initialIKGuess(4).JointPosition = jointStateMsg.Position(5);
    initialIKGuess(5).JointPosition = jointStateMsg.Position(6);
    initialIKGuess(6).JointPosition = jointStateMsg.Position(7);
end


