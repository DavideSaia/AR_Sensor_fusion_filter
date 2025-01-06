%Visual-inertial odometry with the used of drivingScenario tool & 
%Error state Kalman Filter 

clc
clear all
close all
clearvars

% Get the scnario designed with used of DrivingScenarionDesigner & the
% egoVehicle
[scenario,gTCar,estCar,traj] = createDrivingScenario;

sampleRate = 1/scenario.SampleTime;

%Visualize the scene 
chasePlot(gTCar,'Centerline','on')

plot(scenario,"RoadCenters","on","Waypoints","on")
title('Scenario')



%Create Fusion Filter 
filter = insfilterErrorState("IMUSampleRate",sampleRate,'ReferenceFrame','ENU');
%Set the initial state and error state covariance 
filterIntializer(filter,traj);


%Create sensors mounted on vehicle 
[paramsVO,imu] = createSensors(scenario);

%Flag to choose if use VIO or only IMU
useVIO = true;

%SIMULATION SET UP

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

%SIMULATION LOOP
cameraIdx = 1;
for i = 1:numIMUSamples
    % Generate ground truth trajectory values.
    [pos(i,:), orient(i,:), vel(i,:), acc(i,:), angvel(i,:)] = traj();

    % Generate accelerometer and gyroscope measurements from the ground truth
    % trajectory values.
    [accelMeas, gyroMeas] = imu(acc(i,:), angvel(i,:), orient(i));

    % Predict the filter state forward one time step based on the
    % accelerometer and gyroscope measurements.
    predict(filter, accelMeas, gyroMeas); 

    if (1 == mod(i, imuSamplesPerCamera)) && useVIO
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

    posEst(i,3) = 0; %estVehicle floor level

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
if useVIO
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posVO(:,1), posVO(:,2), posVO(:,3), ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'Visual Odometry (VO)', ...
        'Visual-Inertial Odometry (VIO)', 'Location', 'northeast')
else
    plot3(pos(:,1), pos(:,2), pos(:,3), '-.', ...
        posEst(:,1), posEst(:,2), posEst(:,3), ...
        'LineWidth', 3)
    legend('Ground Truth', 'IMU Pose Estimate')
end
view(-90, 90)
title('Vehicle Position')
xlabel('X (m)')
ylabel('Y (m)')
grid on







