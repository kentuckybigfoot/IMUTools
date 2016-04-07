close all
clear all
clc
format long

dataFilePath = 'C:\Users\clk0032\Dropbox\Friction Connection Research\Full Scale Test Data\Data Processing Scripts\';
dataFileName = '[RotationData] FS Testing - ST2 - Test 2 - 04-07-16';
dataFileSavePath = '';
dataFileSaveName = '';
smartPlotting = true;
savePlots = true;

format long

load(sprintf('%s%s', dataFilePath, dataFileName));

%unitquat = sqrt(Aw.^2 + Ax.^2 + Ay.^2 + Az.^2);
%unitquat2 = sqrt(Bw.^2 + Bx.^2 + By.^2 + Bz.^2);

% Consolidate data from Sensor A into variable sensorA and consolidate
% data from Sensor B into variable sensorB. Row 1 = w, row 2 = x,
% row 3 = y, row 4 = z.
sensorA = [Aw Ax Ay Az];
sensorB = [Bw Bx By Bz];

% Check to ensure that we are dealing with normalized quat data.
if any(sensorA > 1 ) | any(sensorB > 1)
    error('Convert to unit quaternions before processing');
end

for r = 1:1:size(sensorA, 1)   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Convert Quaternion to Axis Angle
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/
    % Used as reference
    
    %Get angle
    angle(r,1) = 2*acos(sensorA(r,1)); %sensorA
    angle(r,2) = 2*acos(sensorB(r,1)); %sensorB
    
    %Get and condition denominator
    s1 = sqrt(1-sensorA(r,1)*sensorA(r,1));
    s2 = sqrt(1-sensorB(r,1)*sensorB(r,1));
    
    if s1 < 0.001
        s1 = 1;
    end
    
    if s2 < 0.001
        s2 = 1;
    end
    
    % Save axis data for sensorA
    x(r,1) = sensorA(r,1)/s1;
    y(r,1) = sensorA(r,2)/s1;
    z(r,1) = sensorA(r,3)/s1;
    
    % Save axis data for sensorB
    x(r,2) = sensorB(r,1)/s2;
    y(r,2) = sensorB(r,2)/s2;
    z(r,2) = sensorB(r,3)/s2;
    
    %Get rotation matrix for sensor A
    RA(r,:) = convertQtoM(sensorA(r,:));
    
    %Get rotation matrix for sensor A
    RB(r,:) = convertQtoM(sensorB(r,:));
    %{
    %Roll (gamma) (CC X Axis)
    gamma(r,1) = atan2(R32,R33);
    %Pitch (beta) (CC Y Axis)
    beta(r,1) = atan2(-R31,sqrt(R32^2 + R33^2));
    %Yaw (alpha) (CC Z Axis)
    alpha(r,1) = atan2(R21, R11);
    
    if R11 == 0 && R33 == 0
        disp('Well Shit');
    end
    
    points = [R11 R12 R13; R21 R22 R23; R31 R32 R33]*[1; 1; 1];
    pointsFix(r,:) = [points(1) points(2) points(3)];
    
    points2 = [R11 R12 R13; R21 R22 R23; R31 R32 R33]*points;
    pointsRel(r,:) = [points2(1) points2(2) points2(3)];
    
    R(r,:) = [R11 R12 R13 R21 R22 R23 R31 R32 R33];
    R2(r,:) = [(1 - 2*qw^2 - 2*qz^2) (2*qx*qy - 2*qz*qw) (2*qx*qz + 2*qy*qw) (2*qx*qy + 2*qz*qw) (1 - 2*qx^2 - 2*qz^2) (2*qy*qz - 2*qx*qw) (2*qx*qz - 2*qy*qw) (2*qy*qz + 2*qx*qw) (1 - 2*qx^2 - 2*qy^2)];
    
    if r == 1 || r == 9999 || r == 19999 || r == 29999 || r == 39999 || r == 49999
        r
    end
    %}
end
%{
figure();
start = 1;
limit = length(alpha);
subplot(3,1,1)
plot(start:1:limit, alpha(start:limit,:)*(180/pi));
ylabel('Rotation (degrees)');
xlabel('Index');
title('Yaw Axis Plot (Alpha) (Counter-Clockwise Z Axis Positive Orientation');
grid on
grid minor
subplot(3,1,2)
plot(start:1:limit, beta(start:limit,:)*(180/pi));
ylabel('Rotation (degrees)');
xlabel('Index');
title('Pitch Axis Plot (Beta) (Counter-Clockwise Y Axis Positive Orientation');
grid on
grid minor
subplot(3,1,3)
plot(start:1:limit, gamma(start:limit,:)*(180/pi));
ylabel('Rotation (degrees)');
xlabel('Index');
title('Roll Axis Plot (Gamma) (Counter-Clockwise X Axis Positive Orientation');
grid on
grid minor
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 
1],'Box','off','Visible','off','Units','normalized', 'clipping', 'off');
text(0.5, 1,sprintf('\b start = %d, end = %d',start, limit),'HorizontalAlignment', 'center','VerticalAlignment', 'top')

%plot3(pointsFix(:,1),pointsFix(:,2),pointsFix(:,3))
%}
%{
save '03-13-16-3_LocationData.mat';
rotations
plot3(pointsFix(:,1),pointsFix(:,2),pointsFix(:,3))
figure
plot3(pointsFix(:,1),pointsFix(:,2),pointsFix(:,3))
plot(1:1:length(CalA1), CalB1)
figure
plot(1:1:length(CalA1), CalB1)
plot(7000:1:length(CalA1), alpha(7000:end,:)*(180/pi), 7000:1:length(CalA1), beta(7000:end,:)*(180/pi), 7000:1:length(CalA1), gamma(7000:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
figure
plot(7000:1:length(CalA1), alpha(7000:end,:)*(180/pi), 7000:1:length(CalA1), beta(7000:end,:)*(180/pi), 7000:1:length(CalA1), gamma(7000:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
plot(7350:1:length(CalA1), alpha(7350:end,:)*(180/pi), 7350:1:length(CalA1), beta(7350:end,:)*(180/pi), 7350:1:length(CalA1), gamma(7350:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
figure
plot(7350:1:length(CalA1), alpha(7350:end,:)*(180/pi), 7350:1:length(CalA1), beta(7350:end,:)*(180/pi), 7350:1:length(CalA1), gamma(7350:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
grid on
grid minor
rotations
figure; plot(1:1:length(CalA1), CalA1)
plot(7350:1:length(CalA1), alpha(9750:end,:)*(180/pi), 9750:1:length(CalA1), beta(9750:end,:)*(180/pi), 9750:1:length(CalA1), gamma(9750:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
plot(9750:1:length(CalA1), alpha(9750:end,:)*(180/pi), 9750:1:length(CalA1), beta(9750:end,:)*(180/pi), 9750:1:length(CalA1), gamma(9750:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
figurel plot(9750:1:length(CalA1), alpha(9750:end,:)*(180/pi), 9750:1:length(CalA1), beta(9750:end,:)*(180/pi), 9750:1:length(CalA1), gamma(9750:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
figure; plot(9750:1:length(CalA1), alpha(9750:end,:)*(180/pi), 9750:1:length(CalA1), beta(9750:end,:)*(180/pi), 9750:1:length(CalA1), gamma(9750:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
figure; plot(9900:1:length(CalA1), alpha(9900:end,:)*(180/pi), 9900:1:length(CalA1), beta(9900:end,:)*(180/pi), 9900:1:length(CalA1), gamma(9900:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)')
figure; plot(9900:1:length(CalA1), alpha(9900:end,:)*(180/pi), 9900:1:length(CalA1), beta(9900:end,:)*(180/pi), 9900:1:length(CalA1), gamma(9900:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)', 'Position', 'Best')
figure; plot(9900:1:length(CalA1), alpha(9900:end,:)*(180/pi), 9900:1:length(CalA1), beta(9900:end,:)*(180/pi), 9900:1:length(CalA1), gamma(9900:end,:)*(180/pi)); legend('Yaw (CC Z)','Pitch (CC Y)','Gamma (CC X)', 'Location', 'Best')
grid on
grid minor
plot3(pointsFix(9900:end,1),pointsFix(9900:end,2),pointsFix(9900:end,3))
%}