function [ ] = plotRotations( x, y, name, start, stop, RorD)
%PLOTROTATIONS Plots rotations (yaw, pitch, roll)
%   Detailed explanation goes here

figure();

%Yaw
subplot(3,1,1)
plot(start:1:stop, y(start:stop,1)*(180/pi));
ylabel('Rotation (degrees)');
xlabel('Index');
title('Yaw Axis Plot (Alpha) (Counter-Clockwise Z Axis Positive Orientation');
grid on
grid minor

%Pitch
subplot(3,1,2)
plot(start:1:stop, y(start:stop,2)*(180/pi));
ylabel('Rotation (degrees)');
xlabel('Index');
title('Pitch Axis Plot (Beta) (Counter-Clockwise Y Axis Positive Orientation');
grid on
grid minor

%Roll
subplot(3,1,3)
plot(start:1:stop, y(start:stop,3)*(180/pi));
ylabel('Rotation (degrees)');
xlabel('Index');
title('Roll Axis Plot (Gamma) (Counter-Clockwise X Axis Positive Orientation');
grid on
grid minor

%Add title at the top
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 
1],'Box','off','Visible','off','Units','normalized', 'clipping', 'off');
text(0.5, 1,sprintf('\b start = %d, end = %d',start, stop),'HorizontalAlignment', 'center','VerticalAlignment', 'top')


end

