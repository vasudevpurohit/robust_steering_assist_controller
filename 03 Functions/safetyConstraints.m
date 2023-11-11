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
    %extract the rmpc properties -- horizon and time_interval
    if isempty(n)
        n=0;
        time_horizon=rmpcProps.time_horizon;
        time_step=rmpcProps.time_step;
        V=vehProps.V;
        a=vehProps.a;
        b=vehProps.b;
    else
        %do nothing
    end
    %setting up the ode problem and solving it over the horizon
    tspan=(0:time_step:time_horizon)';
    [t,x] = ode45(@(t,x) odefun(driverProps,vehProps,rmpcProps,refTraj,U,x,t),tspan, x0,[]);
    %calculating the slip angle
    beta=x(:,1);
    r=x(:,2);
    delta=x(:,3)+U;
    psi=x(:,4);
    X=x(:,6);
    Y=x(:,7);
    alpha_fr=beta+(a/V)*r-delta;
    alpha_rr=beta-(b/V)*r;
    %four corners over the horizon
    x_fr=x(:,8);
    x_fl=x(:,9);
    y_fr=x(:,10);
    y_fl=x(:,11);
    x_rr=x(:,12);
    x_rl=x(:,13);
    y_rr=x(:,14);
    y_rl=x(:,15);
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
    x(5)=delta_y
    x(6)=X (global frame,CG)
    x(7)=Y (global frame,CG)
    x(8)=x_fr
    x(9)=x_fl
    x(10)=y_fr
    x(11)=y_fl
    x(12)=x_rr
    x(13)=x_rl
    x(14)=y_rr
    x(15)=y_rl
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
    psi_ref = makima(refTraj.psi_ref,refTraj.X,(x(6)+La));
    %writing down the differential equations'
    tspan=(0:time_step:time_horizon)';
    u=interp1(tspan,U,t);
    dxdt(1,1)=[(Yb/(m*V)) ((Yr/(m*V))-1) (Yd/m*V)]*[x(1) x(2) x(3)+u]';
    dxdt(2,1)=[(Nb/Jz) (Nr/Jz) (Nd/Jz)]*[x(1) x(2) x(3)+u]';
    dxdt(3,1)=[(-1/tau) (-Kd/tau) (-Kd/(La*tau))]*[x(3) x(4) x(5)]'+(Kd/tau)*psi_ref;
    dxdt(4,1)=x(2);
    dxdt(5,1)=V*x(1);
    dxdt(6,1)=V;    
    dxdt(7,1)=V(x(1)+x(2));
    dxdt(8,1)=V;
    dxdt(9,1)=V;
    dxdt(10,1)=V(x(1)+x(2));
    dxdt(11,1)=V(x(1)+x(2));
    dxdt(12,1)=V;
    dxdt(13,1)=V;
    dxdt(14,1)=V(x(1)+x(2));
    dxdt(15,1)=V(x(1)+x(2));
end