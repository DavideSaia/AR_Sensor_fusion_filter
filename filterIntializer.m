%Function for filter start configuration 

function filterIntializer(filt,traj,flag)

%Retrieve the initial position,orientation, and velocity from the 
%trajectory object and reset the internal states 

if flag == '0'
    [pos, orient, vel] = traj();
    reset(traj);
else
    pos = traj.Position;
    orient = quaternion(fliplr(traj.Orientation), 'eulerd', 'ZYX', 'frame');
    vel = traj.Velocity;
end

%Set the initial state values 
filt.State(1:4) = compact(orient(1)).';
filt.State(5:7) = pos(1,:).';
filt.State(8:10)= vel(1,:).';

%Set the gyroscope bias and visual odometry scale factor covariance to 
%large values corresponding to low confidence 
filt.StateCovariance(10:12,10:12) = 1e6;
filt.StateCovariance(end) = 2e2;


end