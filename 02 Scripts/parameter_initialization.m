tbxmanager restorepath
mpt_init
% x`
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
vehProps.V=35/3.6;                  %m/s, vehicle velocity
vehProps.footprint.x_fr=vehProps.a; %m, front right corner X
vehProps.footprint.x_fl=vehProps.a; %m, front left corner X
vehProps.footprint.x_rr=-vehProps.b;%m, rear right corner X
vehProps.footprint.x_rl=-vehProps.b;%m, rear left corner X
vehProps.footprint.y_fr=(-1.75/2);    %m, front right corner Y
vehProps.footprint.y_fl=(1.75/2);     %m, front left corner Y
vehProps.footprint.y_rr=(-1.75/2);    %m, rear right corner Y
vehProps.footprint.y_rl=(1.75/2);     %m, rear left corner Y
%driver properties
driverProps.tau=0.15;               %driver lag time constant
driverProps.Kd=0.15;                %Nm/rad - driver gain
driverProps.La=20;                  %m, look-ahead distance
%rmpc properties
rmpcProps.time_horizon=1.5;          %time horizon for the MPC
rmpcProps.time_step=0.05;           %time-discretization for MPC
rmpcProps.N=rmpcProps.time_horizon/rmpcProps.time_step;
% rmpcProps.Aieq=1;                  %inequality constraint A matrix
% rmpcProps.Bieq=1;                  %inequality constraint B matrix
% rmpcProps.Aeq=1;                   %equality constraint A matrix
% rmpcProps.Beq=1;                   %equality constraint B matrix
rmpcProps.lb=[-0.1*ones(rmpcProps.N,1) 0*ones(rmpcProps.N,1)]; %decision variable lower bound
rmpcProps.ub=[0.1*ones(rmpcProps.N,1) 1e-2*ones(rmpcProps.N,1)]; %decision variable upper bound
rmpcProps.w_deltaU=1.0;
rmpcProps.w_U=1.0;
rmpcProps.w_sv=50.0;
%reference trajectory properties
load('refTraj.mat');
%% checking results
%running the vehicle-driver model without the MPC in loop
driverOnly=sim('vehicle_driver_model.slx');
figure(1)
cla reset
axis equal
obstacle=Polyhedron('A',[1 0;-1 0;0 1;0 -1],'b',[75;-35;1.75;0.75]);
plot(refTraj.X,refTraj.Y,'r--');
hold on
plot(driverOnly.plant(:,6),driverOnly.plant(:,7),'m-');
for i = (1:15:length(driverOnly.footprint))
    patch([driverOnly.footprint(i,1) driverOnly.footprint(i,2) driverOnly.footprint(i,4) driverOnly.footprint(i,3)],[driverOnly.footprint(i,5) driverOnly.footprint(i,6) driverOnly.footprint(i,8) driverOnly.footprint(i,7)],'m');
    alpha(0.2);
end
plot([-10;80],[2;2],'ko--','LineWidth',2.5);
plot([-10;80],[-1;-1],'ko--','LineWidth',2.5);
plot([-10;80],[5;5],'ko--','LineWidth',2.5);
plot(obstacle,'Color','r');
plot(out.X,out.Y,'k-');
% rectangle('Position',[35 -0.75 40 0.75+1.75],'FaceColor',[0 0 1.0],'EdgeColor','b');
for i = (1:15:length(out.footprint))
    patch([out.footprint(i,1) out.footprint(i,2) out.footprint(i,4) out.footprint(i,3)],[out.footprint(i,5) out.footprint(i,6) out.footprint(i,8) out.footprint(i,7)],'black');
end
ylim([-2 6]);
xlabel('X');
ylabel('Y');
% legend("Reference Trajectory","Vehicle Trajectory",'Location','southoutside');