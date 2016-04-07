close all
clear all
clc
format long

dataFilePath     = 'C:\Users\clk0032\Dropbox\Friction Connection Research\Full Scale Test Data\Data Processing Scripts\';
dataFileName     = '[RotationData] FS Testing - ST2 - Test 2 - 04-07-16';
dataFileSavePath = '';
dataFileSaveName = '';
smartPlotting    = true;
savePlots        = true;
saveData         = true;
reverseStr       = ''; %For progress updates

format long

load(sprintf('%s%s', dataFilePath, dataFileName));

% Consolidate data from Sensor A into variable sensorA and consolidate
% data from Sensor B into variable sensorB. Row 1 = w, row 2 = x,
% row 3 = y, row 4 = z.
sensorA = [Aw Ax Ay Az];
sensorB = [Bw Bx By Bz];

% Consolidate calibration data in a similar manner. Row 1 = system, row 2 =
% gyroscope, row 3 = accel, row 4 = mag.
calA = [CalA1 CalA2 CalA3 CalA4];
calB = [CalB1 CalB2 CalB3 CalB4];

%Offset time to zero seconds
offsetTime(:,1) = (time(:)-time(1))/1000;

%Free memory
clearvars Aw Ax Ay Az Bw Bx By Bz CalA1 CalA2 CalA3 CalA4 CalB1 CalB2 CalB3 CalB4 VarName2 VarName7 VarName12 VarName17

% Check to ensure that we are dealing with normalized quat data.
if any(sensorA > 1 ) | any(sensorB > 1)
    error('Convert to unit quaternions before processing');
end

for r = 1:1:size(sensorA, 1)   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Convert Quaternion to Axis Angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [AAA(r,1) AAA(r,2) AAA(r,3) AAA(r,4)] = convertQtoAA(sensorA(r,:));
    [AAB(r,1) AAB(r,2) AAB(r,3) AAB(r,4)] = convertQtoAA(sensorB(r,:));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Convert Quaternion to Rotation Matrix
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    RA(r,:) = convertQtoM(sensorA(r,:));
    RB(r,:) = convertQtoM(sensorB(r,:));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Convert Rotation Matrix to Yaw, Pitch, and Roll (Radians)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Yaw (alpha) (CC Z Axis)
    %Pitch (beta) (CC Y Axis)
    %Roll (gamma) (CC X Axis)
    
    [anglesA(r,1) anglesA(r,2) anglesA(r,3)] = convertMtoA(RA(r,:));
    [anglesB(r,1) anglesB(r,2) anglesB(r,3)] = convertMtoA(RB(r,:));
    
    %Display calculation progress
    percentComplete = 100 * r / size(sensorA,1);
    msg = sprintf('Percent complete: %3.1f', percentComplete);
    fprintf([reverseStr, msg]);
    reverseStr = repmat(sprintf('\b'), 1, length(msg));
end

%Plot rotations for both Sensor A and Sensor B
plotRotations(1, [anglesA(:,1) anglesA(:,2) anglesA(:,3)], dataFileName, 1, length(anglesA), '');
plotRotations(1, [anglesB(:,1) anglesB(:,2) anglesB(:,3)], dataFileName, 1, length(anglesB), '');

%More memory cleaning
clearvars r percentComplete msg reverseStr;

%Left over remnent from dead-reckoning attempts.
%plot3(pointsFix(:,1),pointsFix(:,2),pointsFix(:,3))

