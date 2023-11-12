%main script that populates all the parameters neccessary to run the model
%vehicle properties
vehProps.m=1550;
vehProps.Jz=2000;
vehProps.C1=2*1.6*1e3*180/pi;       % N/rad, per axle
vehProps.C2=2*.5*1e3*180/pi;        % N/rad, per axle
vehProps.CMz1=2*0.13*1e3*180/pi;    % Nm/rad, per axle
vehProps.CMz2=2*0.06*1e3*180/pi;    % Nm/rad, per axle
vehProps.a=1.064;                   % m,front axle distance
vehProps.b=1.596;                   %m, rear axle distance
vehProps.V=70/3.6;                  %m/s, vehicle velocity
vehProps.footprint.x_fr=vehProps.a; %m, front right corner X
vehProps.footprint.x_fl=vehProps.a; %m, front left corner X
vehProps.footprint.x_rr=-vehProps.b;%m, rear right corner X
vehProps.footprint.x_rl=-vehProps.b;%m, rear left corner X
vehProps.footprint.y_fr=1.52/2;     %m, front right corner Y
vehProps.footprint.y_fl=1.52/2;     %m, front left corner Y
vehProps.footprint.y_rr=1.52/2;     %m, rear right corner Y
vehProps.footprint.y_rl=1.52/2;     %m, rear left corner Y
%driver properties
driverProps.tau=0.15;               %driver lag time constant
driverProps.Kd=0.01;                %Nm/rad - driver gain
driverProps.La=22;                  %m, look-ahead distance
%rmpc properties
rmpcProps.Aieq=1;                  %inequality constraint A matrix
rmpcProps.Bieq=1;                  %inequality constraint B matrix
rmpcProps.Aeq=1;                   %equality constraint A matrix
rmpcProps.Beq=1;                   %equality constraint B matrix
rmpcProps.lb=1;                    %decision variable lower bound
rmpcProps.ub=1;                    %decision variable upper bound
%reference trajectory properties
% load('refTraj.mat');
%% checking results
figure(1)
plot(refTraj.X,refTraj.Y,'r--');
hold on
plot(out.X,out.Y,'k-');
xlabel('X');
ylabel('Y');
legend("Reference Trajectory","Vehicle Trajectory",'Location','southoutside');