function updatePose(veh, pos, vel, orient)

pos(:,3) = 0; %Floor Level

veh.Position = pos;
veh.Velocity = vel;
rpy = eulerd(orient, 'ZYX', 'frame');
veh.Yaw = rpy(1);
veh.Pitch = 0;
veh.Roll = rpy(3);
end