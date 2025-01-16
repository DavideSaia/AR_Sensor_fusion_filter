function updatePose(veh, pos, vel, orient)

pos(:,3) = 0; %Floor Level

veh.Position = pos;
veh.Velocity = vel;
rpy = eulerd(orient, 'ZYX', 'frame');
veh.Yaw = rpy(1);%ROtationg along z-axis
veh.Pitch = 0;%Rotation along y-axis
veh.Roll = 0;%Rotation along x-axis
end