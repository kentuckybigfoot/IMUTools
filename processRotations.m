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

for r = 1:1:30%size(sensorA, 1)   
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