function [ alpha beta gamma ] = convertMto( R )
%CONVERTMTOA Convert 3D rotation matrix to rotation angles
%   Input full horizontal rotation matrix, R, and returns alpha (yaw CC pos
%   x axis), beta (pitch CC pos y axis), abd gamma (roll CC pos x axis) all
%   in radians.
%   
%   http://planning.cs.uiuc.edu/node103.html used as reference.

%Yaw (alpha) (CC Z Axis)
alpha = atan2(R(4), R(1));

%Pitch (beta) (CC Y Axis)
beta = atan2(-R(7),sqrt(R(8)^2 + R(9)^2));

%Roll (gamma) (CC X Axis)
gamma = atan2(R(8),R(9));

end

