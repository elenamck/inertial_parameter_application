clc ;
clear all;
close all;
panda = importrobot('Model/urdf/panda_arm_hand.urdf', 'MeshPath',{'Model/meshes/visual'});
% panda.Gravity =  [0, 0, -9.80665];

panda.DataFormat = 'row';
hand = 'hand';
%%%%%%%%%OBSTACLES%%%%%%%%%%%%%
tableRedLength = 0.3;
tableRedWidth = 0.125;
tableRedHeight = 0.22;
tableRedPosition = [0.5,-0.275,0.22];
tableRed = robotics.RigidBody('tableRed');
setFixedTransform(tableRed.Joint, trvec2tform(tableRedPosition))
addBody(panda, tableRed, panda.BaseName);

%%%%%%%%%LOADS%%%%%%%%%%%%%
cylinderHeight = 0.121;
cylinderRadius = 0.038;
cylinderPosition = [tableRedPosition(1)-0.02, tableRedPosition(2)-tableRedWidth/2,tableRedPosition(2)+cylinderHeight/2];
cylinderPositionInHand =[0.0,0.0,cylinderHeight/2]; 
handOrientationCylinder = [0.575 0 0.818 0; -0.789 0.263 0.555 0; -0.215 -0.965 0.150 0; 0 0 0 1];
cylinder = robotics.RigidBody('cylinder');
setFixedTransform(cylinder.Joint, trvec2tform(cylinderPositionInHand))
addBody(panda, cylinder, panda.BaseName);


numWaypoints = 20;
q0 = [-0.35,0.0,0.0,-1.5,0.0,1.5,1.6];
qWaypoints = repmat(q0, numWaypoints, 1);
gik = robotics.GeneralizedInverseKinematics('RigidBodyTree', panda,  'ConstraintInputs', {'cartesian','cartesian','cartesian','position','aiming','orientation','joint'})


tableRedAux = robotics.CartesianBounds(hand);

tableRedAux.Bounds = [tableRedPosition(1),               tableRedPosition(1)+tableRedLength;
                      tableRedPosition(2)-tableRedWidth, tableRedPosition(2);
                      tableRedPosition(3),               tableRedPosition(3)+ tableRedHeight]
distanceTableRed = robotics.CartesianBounds(hand);
distanceTableRed.Bounds = [-inf, tableRedAux.Bounds(1,2) + 0.05;
                           tableRedAux.Bounds(2,2)+0.05 , inf; 
                           tableRedAux.Bounds(3,2) + 0.05, inf];


                 
heightAboveGround = robotics.CartesianBounds(hand);
heightAboveGround.Bounds = [-inf, inf;
                     -inf, inf;
                     0.05, inf]
                 
distanceFromCylinder = robotics.PositionTarget('cylinder')
distanceFromCylinder.ReferenceBody = hand;
distanceFromCylinder.PositionTolerance = 0.001;                
alignWithCylinder = robotics.AimingConstraint('hand');
alignWithCylinder.TargetPoint = cylinderPosition;
limitJointChange = robotics.JointPositionBounds(panda)
fixOrientation = robotics.OrientationTarget(hand,'ReferenceBody','cylinder');
fixOrientation.OrientationTolerance = deg2rad(1);

intermediateDistance = [0.2,0.05,-0.01];
limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 1;

distanceFromCylinder.TargetPosition = intermediateDistance;

[qWaypoints(2,:),solutionInfo] = gik(q0,tableRedAux, distanceTableRed, heightAboveGround,  distanceFromCylinder, alignWithCylinder, fixOrientation, limitJointChange);
        
                   
limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 1;
alignWithCylinder.Weights = 0;

fixOrientation.TargetOrientation = tform2quat(getTransform(panda,qWaypoints(2,:),hand));
finalDistanceFromCylinder = [0.0, 0.001, cylinderHeight/2];
for i = 1:3
    distanceFromCylinderValues(:,i) = linspace(intermediateDistance(i), finalDistanceFromCylinder(i), numWaypoints-1);
end
maxJointChange = deg2rad(10);
for k = 3:numWaypoints
    % Update the target position.
 
    distanceFromCylinder.TargetPosition(:) = distanceFromCylinderValues(k-1,:)
    % Restrict the joint positions to lie close to their previous values.
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange, ...
                               qWaypoints(k-1,:)' + maxJointChange];
    % Solve for a configuration and add it to the waypoints array.
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:),tableRedAux, distanceTableRed, heightAboveGround,  ...
                       distanceFromCylinder, alignWithCylinder, fixOrientation, ...
                       limitJointChange);
end


framerate = 5;
r = robotics.Rate(framerate);
tFinal = 20;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
numFrames = tFinal*framerate;
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';


handPosition = zeros(numFrames,3);
for k = 1:numFrames
    handPosition(k,:) = tform2trvec(getTransform(panda,qInterp(k,:), ...
                                                    hand));
end
figure;
show(panda, qWaypoints(1,:), 'PreservePlot', false);
hold on
exampleHelperPlotCupAndTable(cylinderHeight, cylinderRadius, cylinderPosition);
p = plot3(handPosition(1,1), handPosition(1,2), handPosition(1,3));
hold on
for k = 1:size(qInterp,1)
    show(panda, qInterp(k,:), 'PreservePlot', false);
    p.XData(k) = handPosition(k,1);
    p.YData(k) = handPosition(k,2);
    p.ZData(k) = handPosition(k,3);
    waitfor(r);
end
hold off

% % panda.Gravity =  [0, 0, -9.80665];
% % 
% % 
% % 
% % tStart = 0.5;
% % tDuration = 3;
% % 
% % q0 = [-0.35; 0; 0; -1.5; 0; 1.5; 1.6];
% % q1 = zeros(7,1);
% % 
% % p = exampleHelperPolynomialTrajectory(q0,q1,tDuration);
% % 
% % % open_system('robotSafeTrajectoryTracking.slx')
% % % open_system('robotSafeTrajectoryTracking/Computed Torque Controller');