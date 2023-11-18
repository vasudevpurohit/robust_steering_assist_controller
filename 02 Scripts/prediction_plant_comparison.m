%{
script compares the outputs of the plant model with that of the prediction
model. This is needed to check if the states the same way between the two
and the divergence is not too high.
%}
clear
clc
%initial conditions
x0(1,1:7)=0;
t_final=7.0;
%run the predictionModel
run('parameter_initialization.m');
x=predictionModel(x0,driverProps,vehProps,rmpcProps,refTraj,t_final);
%run the plant simulink model
plant=sim('vehicle_driver_model',t_final);
%plotting and comparing the results
figure(1)
cla reset
plot(x(:,1),'r*-');
hold on
plot(plant.plant(:,1),'bo-');
ylabel('beta');
title('Body Slip Angle');
legend("prediction model","plant");

figure(2)
cla reset
plot(x(:,2),'r*-');
hold on
plot(plant.plant(:,2),'bo-');
ylabel('r');
title('Yaw rate');
legend("prediction model","plant");

figure(3)
cla reset
plot(x(:,3),'r*-');
hold on
plot(plant.plant(:,3),'bo-');
ylabel('delta_dr');
title('Driver Steering Input');
legend("prediction model","plant");

figure(4)
cla reset
plot(x(:,4),'r*-');
hold on
plot(plant.plant(:,4),'bo-');
ylabel('psi');
title('Orientation');
legend("prediction model","plant");

figure(5)
cla reset
plot(x(:,5),'r*-');
hold on
plot(plant.plant(:,5),'bo-');
ylabel('delta_y');
title('delta_y');
legend("prediction model","plant");

figure(6)
cla reset
plot(x(:,6),'r*-');
hold on
plot(plant.plant(:,6),'bo-');
ylabel('X');
title('X');
legend("prediction model","plant");

figure(7)
cla reset
plot(x(:,7),'r*-');
hold on
plot(plant.plant(:,7),'bo-');
ylabel('Y');
title('Y');
legend("prediction model","plant");