%{
This function computes firstly computes the safety constraints and then
imposes a constraint over the permissible safe values. The constraints are
on the four corners of the vehicle to remain within the bounds of the lane,
and the slip angle to stay within the linear region. Both these constraints
are implemented as inequality constraints. Equality constraints remain
empty. The integration carried out is simple by forward euler. Obstacle is
defined in terms of a Polyhedron. Distance between point and set is used to
evaluate how close the point gets to the obstacle.
%}
function [c,ceq]=safetyConstraints_euler(x0,U,driverProps,vehProps,rmpcProps,refTraj)
    persistent time_horizon time_step V a b n obstacle
    persistent x_fr x_fl x_rr x_rl y_fr y_fl y_rr y_rl
    %extract the rmpc properties -- horizon and time_interval
    if isempty(n)
        %initializing the MPT toolbox
        n=0;
        time_horizon=rmpcProps.time_horizon;
        time_step=rmpcProps.time_step;
        V=vehProps.V;
        a=vehProps.a;
        b=vehProps.b;
        x_fr=vehProps.footprint.x_fr;
        x_fl=vehProps.footprint.x_fl;
        x_rr=vehProps.footprint.x_rr;
        x_rl=vehProps.footprint.x_rl;
        y_fr=vehProps.footprint.y_fr;
        y_fl=vehProps.footprint.y_fl;
        y_rr=vehProps.footprint.y_rr;
        y_rl=vehProps.footprint.y_rl;
        obstacle=Polyhedron('A',[1 0;-1 0;0 1;0 -1],'b',[75;-35;1.75;0.75]);
    else
        %do nothing
    end
    %setting up the ode problem and solving it over the horizon
    tspan=(0:time_step:time_horizon)';
    tspan=tspan(2:end);
    x=stateDynamics(x0,U,driverProps,vehProps,rmpcProps,refTraj);
    %calculating the slip angle
    beta=x(2:end,1);
    r=x(2:end,2);
    delta=x(2:end,3)+U(:,1);
    psi=x(2:end,4);
    delta_y=x(2:end,5);
    X=x(2:end,6);
    Y=x(2:end,7);
    sv=U(:,2);
    %slip angle over the horizon
    alpha_fr=beta+(a/V)*r-(delta);
    alpha_rr=beta-(b/V)*r;
    slip_angle_limit=[-(4*pi/180)*ones(length(tspan),1)-sv;(4*pi/180)*ones(length(tspan),1)+sv];
    c_alpha=[-alpha_fr;alpha_fr;-alpha_rr;alpha_rr]-[slip_angle_limit;slip_angle_limit];
    %four corners over the horizon
    x_fr_horizon=x_fr*cos(psi)-y_fr*sin(psi)+X;
    x_fl_horizon=x_fl*cos(psi)-y_fl*sin(psi)+X;
    x_rr_horizon=x_rr*cos(psi)-y_rr*sin(psi)+X;
    x_rl_horizon=x_rl*cos(psi)-y_rl*sin(psi)+X;
    y_fr_horizon=x_fr*sin(psi)+y_fr*cos(psi)+Y;
    y_fl_horizon=x_fl*sin(psi)+y_fl*cos(psi)+Y;
    y_rr_horizon=x_rr*sin(psi)+y_rr*cos(psi)+Y;
    y_rl_horizon=x_rl*sin(psi)+y_rl*cos(psi)+Y;
    corners(1,:)=[x_fr_horizon' x_fl_horizon' x_rr_horizon' x_rl_horizon'];
    corners(2,:)=[y_fr_horizon' y_fl_horizon' y_rr_horizon' y_rl_horizon'];
    for i = (1:4*length(x_fr_horizon))
        distanceStruct=obstacle.distance(corners(:,i));
        corner_distances(i,1)=distanceStruct.dist;
    end
    c_corners=-corner_distances;
    c=[c_alpha;c_corners];
    %creating the c-matrix for inequality constraints
    %no equality constraints
    ceq=[];
end

%{
Euler integration to get all the states over the horizon
%}
function x=stateDynamics(x0,U,driverProps,vehProps,rmpcProps,refTraj)
    %{
    x(1)=body slip angle
    x(2)=yaw rate
    x(3)=driver steering angle
    x(4)=heading angle
    x(5)=delta_y
    x(6)=X (global frame,CG)
    x(7)=Y (global frame,CG)
    %}
    persistent n time_step time_horizon C1 C2 a b V CMz1 CMz2 m Jz Kd tau La
    if isempty(n)
        n=0;
        C1=vehProps.C1;
        C2=vehProps.C2;
        a=vehProps.a;
        b=vehProps.b;
        V=vehProps.V;
        CMz1=vehProps.CMz1;
        CMz2=vehProps.CMz2;
        m=vehProps.m;
        Jz=vehProps.Jz;
        Kd=driverProps.Kd;
        tau=driverProps.tau;
        La=driverProps.La;
        time_step=rmpcProps.time_step;
        time_horizon=rmpcProps.time_horizon;
    else
        %do nothing
    end
    %calculating the derivatives of stability
    Yb=-(C1+C2);
    Yr=(-1/V)*(a*C1-b*C2);
    Yd=C1;
    Nb=-a*C1+b*C2+CMz1++CMz2;
    Nr=(1/V)*(-a*a*C1-b*b*C2+a*CMz1-b*CMz2);
    Nd=a*C1-CMz1;
    %finding the state trajectory via Euler integration
    tspan=(0:time_step:time_horizon)';
    x=zeros(length(tspan),length(x0));  %kx7 matrix
    x(1,:)=x0;                          %first row will be initial conditions
    %calculating the psi_ref needed to determine the driver steering angle
    for i = (1:length(tspan)-1)
        psi_ref = makima(refTraj.X,refTraj.psi_ref,(x(i,6)+La));
        dbeta_dt=[(Yb/(m*V)) ((Yr/(m*V))-1) (Yd/(m*V))]*[x(i,1) x(i,2) x(i,3)+U(i,1)]';
        dr_dt=[(Nb/Jz) (Nr/Jz) (Nd/Jz)]*[x(i,1) x(i,2) x(i,3)+U(i,1)]';
        ddelta_dt=[(-1/tau) (-Kd/tau) (-Kd/(La*tau))]*[x(i,3) x(i,4) x(i,5)]'+(Kd/tau)*psi_ref;
        dpsi_dt=x(i,2);
        ddeltay_dt=V*sin(x(i,1));
        dX_dt=V*cos(x(i,1)+x(i,4));
        dY_dt=V*sin(x(i,1)+x(i,4));
        %euler integration to get the new states
        x(i+1,1)=x(i,1)+dbeta_dt*time_step;
        x(i+1,2)=x(i,2)+dr_dt*time_step;
        x(i+1,3)=x(i,3)+ddelta_dt*time_step;
        x(i+1,4)=x(i,4)+dpsi_dt*time_step;
        x(i+1,5)=x(i,5)+ddeltay_dt*time_step;
        x(i+1,6)=x(i,6)+dX_dt*time_step;
        x(i+1,7)=x(i,7)+dY_dt*time_step;
    end
end