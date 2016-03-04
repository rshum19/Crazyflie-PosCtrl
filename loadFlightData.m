clear all; clc; close all;
% LOAD DATA
load('FlightData/CF_flight_22.mat')

figure
subplot(3,1,1)
plot(PosDes.time,PosDes.signals.values(:,1),Pos.time,Pos.signals.values(:,1))
subplot(3,1,2)
plot(PosDes.time,PosDes.signals.values(:,2),Pos.time,Pos.signals.values(:,2))
subplot(3,1,3)
plot(PosDes.time,PosDes.signals.values(:,3),Pos.time,Pos.signals.values(:,3))

figure
plot(PosDes.time,PosDes.signals.values(:,2),Pos.time,Pos.signals.values(:,2))
xlabel('Time [sec]')
ylabel('Height, Y-position [m]')
title('Height vs Time')
legend('Desired','Actual')

figure
plot(PosDes.time,Thrust_cmd(:,2))
xlabel('Time [sec]')
ylabel('Thrust command [%]')