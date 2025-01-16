%Visual-inertial odometry with the used of drivingScenario tool & 
%Error state Kalman Filter 

clc
close all
clearvars

%VARIABLE DECLARATION FOR BOTH EXPERIMENTS -------------------------------

% Get the scenario designed with used of DrivingScenarionDesigner & the
% egoVehicle
[scenario,gTCar,estCar,traj] = createDrivingScenario;

%sampleRate as fuction of scenario sampleTime
sampleRate = 1/scenario.SampleTime;  
                                      
%Create Fusion Filter 
filter = insfilterErrorState("IMUSampleRate",sampleRate,...
                             'ReferenceFrame','ENU');


%Visualize the scene -------------------------------------------

hFigure = figure;
hFigure.Position(3) = 1000;

hPanel1 = uipanel(hFigure,'Units','normalized',Position=[0 0 1/2 1],...
                  Title='Scenario Plot');
hPanel2 = uipanel(hFigure,'Units','normalized',Position=[1/2 0 1/2 1],...
                  Title='Chase Plot');

hAxes1 = axes('Parent',hPanel1);
hAxes2 = axes('Parent',hPanel2);

plot(scenario,"RoadCenters","on","Waypoints","on",'Parent',hAxes1);
chasePlot(gTCar,'Centerline','on','Parent',hAxes2,'Meshes','on');

%--------------------------------------------------------------------------


%SIMULATION SET UP USING SMOOTH TRAJECTORY --------------------------------

%Get the ground truth state
groundTruth = state(gTCar);


%Create sensors mounted on vehicle 
[paramsVO,imu, mountingLocationIMU,...
    mountingAnglesIMU] = createSensors(scenario);


%Set the initial state and error state covariance 
filterIntializer(filter,groundTruth,'1');

% Convert orientation offset from Euler angles to quaternion.
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');

%Samples per camera 
imuSamplesPerCamera = 4;
numIMUSamples = 3368;%get from a dummy simulation
numCameraSamples = numIMUSamples/4;

% Preallocate data arrays for plotting results.
[pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst] ...
    = preallocateDataSimulation(numIMUSamples, numCameraSamples);

% Set measurement noise parameters for the visual odometry fusion.
RposVO = 0.1;
RorientVO = 0.1;

%Index
i=1;%loop
cameraIdx = 1;%camera sample 


%Click a button to start
pause()
while advance(scenario)
    groundTruth = state(gTCar);


    % Unpack the ground truth struct by converting the orientations from
    % Euler angles to quaternions and converting angular velocities form
    % degrees per second to radians per second.
    pos(i,:) = groundTruth.Position;
    orient(i,:) = quaternion(fliplr(groundTruth.Orientation),...
                             'eulerd', 'ZYX', 'frame');
    vel(i,:) = groundTruth.Velocity;
    acc(i,:) = groundTruth.Acceleration;
    angvel(i,:) = deg2rad(groundTruth.AngularVelocity);


    % Convert motion quantities from vehicle frame to IMU frame.
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = transformMotion( ...
        mountingLocationIMU,orientVeh2IMU, ...
        pos(i,:),orient(i),vel(i,:),acc(i,:),angvel(i,:));


    % Generate accelerometer and gyroscope measurements from the 
    % ground truth trajectory values.
    [accel, gyro] = imu(accIMU,angvelIMU,orientIMU);


    %Predict the filter state forward one time step based on the
    % accelerometer and gyroscope measurements.
    predict(filter, accel, gyro); 


    if (1 == mod(i, imuSamplesPerCamera)) 
        % Generate a visual odometry pose estimate from the ground truth
        % values and the visual odometry model.
        [posVO(cameraIdx,:), orientVO(cameraIdx,:), paramsVO] = ...
            visualOdometryModel(pos(i,:), orient(i,:), paramsVO);

        % Correct filter state based on visual odometry data.
        fusemvo(filter, posVO(cameraIdx,:), RposVO, ...
            orientVO(cameraIdx), RorientVO);

        cameraIdx = cameraIdx + 1;
    end

    %Get the pose estimated by the filter 
    [posEst(i,:), orientEst(i,:), velEst(i,:)] = pose(filter);

    % Update estimated vehicle pose.
    updatePose(estCar, posEst(i,:), velEst(i,:), orientEst(i));

    % Update driving scenario visualization.
    updatePlots(scenario)
    drawnow limitrate;

    i=i+1;%Update index

end

%PLOT RESULTS
figure

plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
    posVO(:,1), posVO(:,2), posVO(:,3), ...
    posEst(:,1), posEst(:,2), posEst(:,3), ...
    'LineWidth', 3)
legend('Ground Truth', 'Visual Odometry (VO)', ...
    'Visual-Inertial Odometry (VIO)', 'Location', 'northeast')

view(-90, 90)
title('Vehicle Position')
xlabel('X (m)')
ylabel('Y (m)')
grid on

%------------------------------------------------------------------------

%Click a button to start
pause();

%SIMULATION SET UP USING WAYPOINT TRAJECTORY ------------------------------

%Create sensors mounted on vehicle 
[paramsVO,imu, mountingLocationIMU,...
    mountingAnglesIMU] = createSensors(scenario);


%Set the initial state and error state covariance 
filterIntializer(filter,traj,'0');


% Run the simulation for 120 seconds.
numSecondsToSimulate = 120;
numIMUSamples = numSecondsToSimulate * sampleRate;

% Define the visual odometry sampling rate.
imuSamplesPerCamera = 4;
numCameraSamples = ceil(numIMUSamples / imuSamplesPerCamera);

% Preallocate data arrays for plotting results.
[pos, orient, vel, acc, angvel, ...
    posVO, orientVO, ...
    posEst, orientEst, velEst] ...
    = preallocateDataSimulation(numIMUSamples, numCameraSamples);

% Set measurement noise parameters for the visual odometry fusion.
RposVO = 0.1;
RorientVO = 0.1;

% Convert orientation offset from Euler angles to quaternion.
orientVeh2IMU = quaternion(mountingAnglesIMU,'eulerd','ZYX','frame');

%SIMULATION LOOP
cameraIdx = 1;
for i = 1:numIMUSamples
    % Generate ground truth trajectory values.
    [pos(i,:), orient(i,:), vel(i,:), acc(i,:), angvel(i,:)] = traj();


    % Convert motion quantities from vehicle frame to IMU frame.
    [posIMU,orientIMU,velIMU,accIMU,angvelIMU] = transformMotion( ...
        mountingLocationIMU,orientVeh2IMU, ...
        pos(i,:),orient(i),vel(i,:),acc(i,:),angvel(i,:));

    % Generate accelerometer and gyroscope measurements from the 
    % ground truth trajectory values.
    [accelMeas, gyroMeas] = imu(accIMU,angvelIMU,orientIMU);

    % Predict the filter state forward one time step based on the
    % accelerometer and gyroscope measurements.
    predict(filter, accelMeas, gyroMeas); 

    if (1 == mod(i, imuSamplesPerCamera)) 
        % Generate a visual odometry pose estimate from the ground truth
        % values and the visual odometry model.
        [posVO(cameraIdx,:), orientVO(cameraIdx,:), paramsVO] = ...
            visualOdometryModel(pos(i,:), orient(i,:), paramsVO);

        % Correct filter state based on visual odometry data.
        fusemvo(filter, posVO(cameraIdx,:), RposVO, ...
            orientVO(cameraIdx), RorientVO);

        cameraIdx = cameraIdx + 1;
    end

    [posEst(i,:), orientEst(i,:), velEst(i,:)] = pose(filter);

    % Update estimated vehicle pose.
    updatePose(estCar, posEst(i,:), velEst(i,:), orientEst(i));

    % Update ground truth vehicle pose.
    updatePose(gTCar, pos(i,:), vel(i,:), orient(i));


    % Update driving scenario visualization.
    updatePlots(scenario);
    drawnow limitrate;
end

%PLOT RESULTS
figure

plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
    posVO(:,1), posVO(:,2), posVO(:,3), ...
    posEst(:,1), posEst(:,2), posEst(:,3), ...
    'LineWidth', 3)
legend('Ground Truth', 'Visual Odometry (VO)', ...
    'Visual-Inertial Odometry (VIO)', 'Location', 'northeast')

view(-90, 90)
title('Vehicle Position')
xlabel('X (m)')
ylabel('Y (m)')
grid on
%-------------------------------------------------------------------------













