% tbxmanager restorepath
% mpt_init
% clear
% clc
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
vehProps.footprint.y_fr=(-1.75/2);    %m, front right corner Y
vehProps.footprint.y_fl=(1.75/2);     %m, front left corner Y
vehProps.footprint.y_rr=(-1.75/2);    %m, rear right corner Y
vehProps.footprint.y_rl=(1.75/2);     %m, rear left corner Y
%driver properties
driverProps.tau=0.15;               %driver lag time constant
driverProps.Kd=0.09;                %Nm/rad - driver gain
driverProps.La=22;                  %m, look-ahead distance
%rmpc properties
rmpcProps.time_horizon=0.75;        %time horizon for the MPC
rmpcProps.time_step=0.05;           %time-discretization for MPC
rmpcProps.N=rmpcProps.time_horizon/rmpcProps.time_step;
a=-0.1;
b=0.1;
t_total=(0:rmpcProps.time_step:3.5);
for i = (1:1)
    w=a+(b-a)*rand(length(t_total),1);
    uncertainties_array(i,1)=timeseries(w,t_total);
end
% rmpcProps.Aieq=1;                  %inequality constraint A matrix
% rmpcProps.Bieq=1;                  %inequality constraint B matrix
% rmpcProps.Aeq=1;                   %equality constraint A matrix
% rmpcProps.Beq=1;                   %equality constraint B matrix
% rmpcProps.lb=[-0.16*ones(rmpcProps.N,1) 0*ones(rmpcProps.N,1)]; %decision variable lower bound
% rmpcProps.ub=[0.16*ones(rmpcProps.N,1) 1e-2*ones(rmpcProps.N,1)]; %decision variable upper bound
rmpcProps.w_deltaU=50.0;
rmpcProps.w_U=50.0;
rmpcProps.w_sv=50.0;
rmpcProps.w_alpha=30.0;
rmpcProps.w_obstacle=1.0;
rmpcProps.w_lane=1.0;
rmpcProps.K=[0.833 0.114 0.821 1.272 0.317];
%reference trajectory properties
load('refTraj.mat');
%%
theta=linspace(0.1,1,5)';   %control scaling
alpha_c=linspace(0.1,0.15,5)';   %state scaling
for i = (1:length(theta))
    for j = (1:length(alpha_c))
        rmpcProps.alpha=alpha_c(j,1);
        for k = (1:length(uncertainties_array))
            rmpcProps.lb=[-theta(i,1)*0.16*ones(rmpcProps.N,1) 0*ones(rmpcProps.N,1)]; %decision variable lower bound
            rmpcProps.ub=[theta(i,1)*0.16*ones(rmpcProps.N,1) 1e-2*ones(rmpcProps.N,1)]; %decision variable upper bound
            uncertainty=uncertainties_array(k,1);
            mpc_control=sim("main_model",3.5);
            mpc_control_tube=sim("main_model_tube_mpc",3.5);
            filename=append("D:\Fall 2023\01 Robust Predictive Control\02 Project\robust_steering_assist_controller\07 Results New\result_theta_",num2str(theta(i,1)),"_alpha_",num2str(alpha_c(j,1)),"_w_",num2str(k+3),".mat");
            save(filename);
        end
    end
end
%% checking results
%running the vehicle-driver model without the MPC in loop
driverOnly=sim('vehicle_driver_model.slx');
figure(1)
% cla reset
axis equal
obstacle=Polyhedron('A',[1 0;-1 0;0 1;0 -1],'b',[75;-35;1.75;0.75]);
upper_lane=Polyhedron('A',[1 0;0 1;-1 0;0 -1],'b',[80;6;10;-5]);
lower_lane=Polyhedron('A',[1 0;0 1;-1 0;0 -1],'b',[80;-1;10;2]);
plot(refTraj.X,refTraj.Y,'r--');
hold on
plot(driverOnly.plant(:,6),driverOnly.plant(:,7),'m-');
for i = (1:5:length(driverOnly.footprint))
    patch([driverOnly.footprint(i,1) driverOnly.footprint(i,2) driverOnly.footprint(i,4) driverOnly.footprint(i,3)],[driverOnly.footprint(i,5) driverOnly.footprint(i,6) driverOnly.footprint(i,8) driverOnly.footprint(i,7)],'m');
    alpha_c(0.2);
end
plot([-10;80],[2;2],'ko--','LineWidth',2.5);
plot([-10;80],[-1;-1],'ko--','LineWidth',2.5);
plot([-10;80],[5;5],'ko--','LineWidth',2.5);
plot(obstacle,'Color','r');
plot(upper_lane,'Color','y');
plot(lower_lane,'Color','y');
plot(mpc_control.X,mpc_control.Y,'k-');
plot(mpc_control_tube.X_a,mpc_control_tube.Y_a,'r-');
% rectangle('Position',[35 -0.75 40 0.75+1.75],'FaceColor',[0 0 1.0],'EdgeColor','b');
for i = (1:5:length(mpc_control.footprint))
    patch([mpc_control.footprint(i,1) mpc_control.footprint(i,2) mpc_control.footprint(i,4) mpc_control.footprint(i,3)],[mpc_control.footprint(i,5) mpc_control.footprint(i,6) mpc_control.footprint(i,8) mpc_control.footprint(i,7)],'black');
    patch([mpc_control_tube.footprint_a(i,1) mpc_control_tube.footprint_a(i,2) mpc_control_tube.footprint_a(i,4) mpc_control_tube.footprint_a(i,3)],[mpc_control_tube.footprint_a(i,5) mpc_control_tube.footprint_a(i,6) mpc_control_tube.footprint_a(i,8) mpc_control_tube.footprint_a(i,7)],'red');
    alpha_c(0.2);
end
% ylim([-2 6]);
xlabel('X');
ylabel('Y');
% legend("Reference Trajectory","Vehicle Trajectory",'Location','southoutside');
%% checking the spread of the trajectories between the tube based and nominal MPC
for i = (1:10)
    load_file_nom=append("D:\Fall 2023\01 Robust Predictive Control\02 Project\robust_steering_assist_controller\05 Results\result_",num2str(i));
    load_file_act=append("D:\Fall 2023\01 Robust Predictive Control\02 Project\robust_steering_assist_controller\05 Results\result_tube_",num2str(i));
    nom=load(load_file_nom,"mpc_control");
    act=load(load_file_act,"mpc_control_tube");
    figure(1)
    plot(nom.mpc_control.X,nom.mpc_control.Y,'k-');
    hold on
    plot(act.mpc_control_tube.X_a,act.mpc_control_tube.Y_a,'r-');
end