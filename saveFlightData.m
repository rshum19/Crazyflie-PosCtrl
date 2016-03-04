% Plot & save data
gains.Kpz = Kpz*70;
gains.Kdz = Kdz*30;
Description = 'Hover tuning Test -- stable: omega*1.26 ---vid41';
save('FlightData/CF_flight_28.mat','RealTime','Pos','PosDes','Vel','gains','Thrust_cmd','Description')

% Position
figure
subplot(3,1,1)
plot(RealTime,PosDes.signals.values(:,1),RealTime,Pos.signals.values(:,1))
subplot(3,1,2)
plot(RealTime,PosDes.signals.values(:,2),RealTime,Pos.signals.values(:,2))
subplot(3,1,3)
plot(RealTime,PosDes.signals.values(:,3),RealTime,Pos.signals.values(:,3))

figure
plot(RealTime,PosDes.signals.values(:,2),RealTime,Pos.signals.values(:,2))
xlabel('Time [sec]')
ylabel('Height, Y-position [m]')
title('Height vs Time')
legend('Desired','Actual')

% Velocity

figure
subplot(3,1,1)
plot(RealTime,Vel.signals.values(:,1));
xlabel('Time [sec]')
ylabel('Velocity, X [m/s]')
subplot(3,1,2)
plot(RealTime,Vel.signals.values(:,2));
xlabel('Time [sec]')
ylabel('Velocity, Y [m/s]')
subplot(3,1,3)
plot(RealTime,Vel.signals.values(:,3));
xlabel('Time [sec]')
ylabel('Velocity, Z [m/s]')



figure
plot(RealTime,Thrust_cmd(:,2))
xlabel('Time [sec]')
ylabel('Thrust command [%]')