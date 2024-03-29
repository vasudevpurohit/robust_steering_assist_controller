%{
This function computes firstly computes the safety constraints and then
imposes a constraint over the permissible safe values. The constraints are
on the four corners of the vehicle to remain within the bounds of the lane,
and the slip angle to stay within the linear region. Both these constraints
are implemented as inequality constraints. Equality constraints remain
empty. The integration carried out is simple by forward euler.
%}
function [c,ceq]=safetyConstraints(x0,U,driverProps,vehProps,rmpcProps,refTraj)
    persistent time_horizon time_step V a b n
    persistent x_fr x_fl x_rr x_rl y_fr y_fl y_rr y_rl
    %extract the rmpc properties -- horizon and time_interval
    if isempty(n)
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
    else
        %do nothing
    end
    %setting up the ode problem and solving it over the horizon
    tspan=(0:time_step:time_horizon)';
    tspan=tspan(2:end);
    [t,x] = ode45(@(t,x) odefun(driverProps,vehProps,rmpcProps,refTraj,U,x,t),tspan, x0,[]);
    %calculating the slip angle
    beta=x(:,1);
    r=x(:,2);
    delta=x(:,3)+U(:,1);
    psi=x(:,4);
    X=x(:,5);
    Y=x(:,6);
    sv=U(:,2);
    %slip angle over the horizon
    alpha_fr=beta+(a/V)*r-delta;
    alpha_rr=beta-(b/V)*r;
    slip_angle_limit=[-0.4*ones(length(tspan),1)-sv;0.4*ones(length(tspan),1)+sv];
    c=[-alpha_fr;alpha_fr;-alpha_rr;alpha_rr]-[slip_angle_limit;slip_angle_limit];
    %four corners over the horizon
    x_fr_horizon=x_fr*cos(psi)-y_fr*sin(psi)+X;
    x_fl_horizon=x_fl*cos(psi)-y_fl*sin(psi)+X;
    x_rr_horizon=x_rr*cos(psi)-y_rr*sin(psi)+X;
    x_rl_horizon=x_rl*cos(psi)-y_rl*sin(psi)+X;
    y_fr_horizon=x_fr*sin(psi)+y_fr*cos(psi)+Y;
    y_fl_horizon=x_fl*sin(psi)+y_fl*cos(psi)+Y;
    y_rr_horizon=x_rr*sin(psi)+y_rr*cos(psi)+Y;
    y_rl_horizon=x_rl*sin(psi)+y_rl*cos(psi)+Y;
    %creating the c-matrix for inequality constraints
    %no equality constraints
    ceq=[];
end

%{
ordinary differential equations for the closed loop vehicle and driver
model
%}
function dxdt=odefun(driverProps,vehProps,rmpcProps,refTraj,U,x,t)
    persistent n C1 C2 a b V CMz1 CMz2 m Jz Kd tau La time_step time_horizon
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
    %{
    x(1)=body slip angle
    x(2)=yaw rate
    x(3)=driver steering angle
    x(4)=heading angle
    x(5)=X (global frame,CG)
    x(6)=Y (global frame,CG)
    %}
    %calculating the derivatives of stability
    Yb=-(C1+C2);
    Yr=(-1/V)*(a*C1-b*C2);
    Yd=C1;
    Nb=-a*C1+b*C2+CMz1++CMz2;
    Nr=(1/V)*(-a*a*C1-b*b*C2+a*CMz1-b*CMz2);
    Nd=a*C1-CMz1;
    %calculating what the psi_ref is -- modified akima interpolation same
    %used in the simulink model
    psi_ref = makima(refTraj.X,refTraj.psi_ref,(x(6)+La));
    %writing down the differential equations'
    tspan=(0:time_step:time_horizon)';
    tspan=tspan(2:end);
    u=interp1(tspan,U(:,1),t);
    dxdt(1,1)=[(Yb/(m*V)) ((Yr/(m*V))-1) (Yd/m*V)]*[x(1) x(2) x(3)+u]';
    dxdt(2,1)=[(Nb/Jz) (Nr/Jz) (Nd/Jz)]*[x(1) x(2) x(3)+u]';
    dxdt(3,1)=[(-1/tau) (-Kd/tau) (-Kd/(La*tau))]*[x(3) x(4) x(5)]'+(Kd/tau)*psi_ref;
    dxdt(4,1)=x(2);
    dxdt(5,1)=V;    
    dxdt(6,1)=V*(x(1)+x(2));
end