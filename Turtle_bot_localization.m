clc
close all
%%
%Connect to the TurtleBot in Gazebo
ipaddress = '192.168.1.2';
rosinit(ipaddress,11311);
%% Load the Map of the Simulation World
load officemap.mat
show(map)
%%
% %Convert PGM Image to Map
% %Import image using imread.
% image = imread('playground.pgm');
% %imshow(image)
% 
% %Unknown areas (gray) should be removed and treated as free space...
% ...Create a logical matrix based on a threshold. 
% imageBW = image < 100;
% %imshow(imageBW)
% 
% %Create binaryOccupancyMap object using adjusted map image.
% map = binaryOccupancyMap(imageBW);
% show(map)
%% Setup the Laser Sensor Model and TurtleBot Motion Model
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
%%
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;
%%
% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'base_link','camera_link',5);
sensorTransform = getTransform(tftree,'base_link','camera_link');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
%% Receiving Sensor Measurements and Sending Velocity Commands
% Create ROS subscribers for retrieving sensor and odometry measurements...
...from TurtleBot
laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
% Create ROS publisher for sending out velocity commands to TurtleBot
[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
%% Initialize AMCL Object
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
% Assign the MotionModel and SensorModel properties in the amcl object.
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
%% Configure AMCL Object for Localization with Initial Pose Estimate
amcl.GlobalLocalization = true;
amcl.ParticleLimits = [250 6000];
%% Setup Helper for Visualization and Driving TurtleBot
visualizationHelper = ExampleHelperAMCLVisualization(map);
wanderHelper = ...
    ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%% Localization Procedure
numUpdates = 50;
Truepose = zeros(numUpdates,3);
Predpose = zeros(numUpdates,3);
errors = zeros(numUpdates,1);
i = 1;
while i < numUpdates+1
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;
    
    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);    
  
    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);
    groundTruthPose = ExampleHelperAMCLGazeboTruePose();
    %Truepose(i,3) = groundTruthPose;
    %Predpose(i,3) = estimatedPose;
    error = immse(estimatedPose, groundTruthPose);
    errors(i,1)=error;
      % Drive robot to next pose.
    %Set up VFH object for obstacle avoidance.
laserSub = rossubscriber('/scan');
[velPub2, velMsg2] = rospublisher('/mobile_base/commands/velocity');

vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.DistanceLimits = [0.05 1];
vfh.RobotRadius = 0.1;
vfh.MinTurningRadius = 0.5;
vfh.SafetyDistance = 0.2;

targetDir = 0;

rate = rateControl(10);

% Drive the robot by sending a message containing the angular velocity...
...and the desired linear velocity using the ROS publisher.
    while rate.TotalElapsedTime < 0.5

	% Get laser scan data
	laserScan = receive(laserSub);
	ranges = double(laserScan.Ranges);
	angles = double(laserScan.readScanAngles);
 
	% Create a lidarScan object from the ranges and angles
     scan = lidarScan(ranges,angles);
        
	% Call VFH object to computer steering direction
	steerDir = vfh(scan, targetDir);  
    
	% Calculate velocities
	if ~isnan(steerDir) % If steering direction is valid
		desiredV = 0.2;
		w = exampleHelperComputeAngularVelocity(steerDir, 1);
	else % Stop and search for valid direction
		desiredV = 0.0;
		w = 0.5;
	end

	% Assign and send velocity commands
	velMsg2.Linear.X = desiredV;
	velMsg2.Angular.Z = w;
	velPub2.send(velMsg2);
    end
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end
    
end
figure
n = [1:i-1]';
plot(n,errors);
title('Mean quare error of estimated and true pose')
xlabel('Updates') 
ylabel('Mean square error') 
%% Stop the TurtleBot and Shutdown ROS in MATLAB
stop(wanderHelper);
rosshutdown;