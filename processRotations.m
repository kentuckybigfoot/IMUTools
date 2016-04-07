clear all
load '04-01-16-1_LocationData.mat';
format long

unitquat = sqrt(Bw.^2 + Bx.^2 + By.^2 + Bz.^2);
%unitquat2 = sqrt(Bw.^2 + Bx.^2 + By.^2 + Bz.^2);

for r = 1:1:length(unitquat)
    qwn(r,1) = Bw(r,1)/unitquat(r,1);
    qxn(r,1) = Bx(r,1)/unitquat(r,1);
    qyn(r,1) = By(r,1)/unitquat(r,1);
    qzn(r,1) = Bz(r,1)/unitquat(r,1);
    
    qw = qwn(r,1);
    qx = qxn(r,1);
    qy = qyn(r,1);
    qz = qzn(r,1);
    
    angle(r,1) = 2*acos(qw); 
    s = sqrt(1-qw*qw);
    
    if s < 0.001
        s = 1;
    end
    
    x(r,1) = qx/s;
    y(r,1) = qy/s;
    z(r,1) = qz/s;
    
    R11 = (1 - 2*qy^2 - 2*qz^2);
    R12 = (2*qx*qy - 2*qz*qw);
    R13 = (2*qx*qz + 2*qy*qw);
    R21 = (2*qx*qy + 2*qz*qw);
    R22 = (1 - 2*qx^2 - 2*qz^2);
    R23 = (2*qy*qz - 2*qx*qw);
    R31 = (2*qx*qz - 2*qy*qw);
    R32 = (2*qy*qz + 2*qx*qw);
    R33 = (1 - 2*qx^2 - 2*qy^2);
    
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
end

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
%{
function [ M ] = convertQtoM( q )
%CONVERTQTOM Converts a quaternion (q = (w, x, y, z)) to a rotation matrix.
%            d1 = M(1,:), d2 = M(2,:), d3 = M(3,:).
    M = zeros(3,3);
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
    M(1,1) = q1^2-q2^2-q3^2+q0^2; M(1,2) = 2*(q1*q2-q0*q3);      M(1,3) = 2*(q1*q3+q0*q2); 
    M(2,1) = 2*(q1*q2+q0*q3);     M(2,2) = -q1^2+q2^2-q3^2+q0^2; M(2,3) = 2*(q2*q3-q0*q1); 
    M(3,1) = 2*(q1*q3-q0*q2);     M(3,2) = 2*(q2*q3+q0*q1);      M(3,3) = -q1^2-q2^2+q3^2+q0^2;

end

convertQtoM([qwn(1700), qxn(1700), qyn(1700), qzn(1700)]);
%}