function cost=costFunc_3_tube(x0,U0,U,rmpcProps,vehProps,driverProps,refTraj)
    %{
    Calculates the cost over the horizon. The U is a matrix over the
    horizon with each row containing the u and epsilon values needed to
    calculate the cost. U0 is used to calculate the delta at the very first
    time step
    %}
    persistent n N w_deltaU w_U w_sv w_alpha w_obstacle w_lane a b V
    persistent x_fr x_fl x_rr x_rl y_fr y_fl y_rr y_rl alpha
    if isempty(n)
        n=0;
        N=rmpcProps.N;
        w_deltaU=rmpcProps.w_deltaU;
        w_U=rmpcProps.w_U;
        w_sv=rmpcProps.w_sv;
        w_alpha=rmpcProps.w_alpha;
        w_obstacle=rmpcProps.w_obstacle;
        w_lane=rmpcProps.w_lane;
        x_fr=vehProps.footprint.x_fr;
        x_fl=vehProps.footprint.x_fl;
        x_rr=vehProps.footprint.x_rr;
        x_rl=vehProps.footprint.x_rl;
        y_fr=vehProps.footprint.y_fr;
        y_fl=vehProps.footprint.y_fl;
        y_rr=vehProps.footprint.y_rr;
        y_rl=vehProps.footprint.y_rl;
        a=vehProps.a;
        b=vehProps.b;
        V=vehProps.V;
        alpha=rmpcProps.alpha;
    else
        %do nothing
    end
    
    stage_cost=zeros(N,1);
    x=stateDynamics(x0,U,driverProps,vehProps,rmpcProps,refTraj);
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
%     alpha_cost=sum((max(0,(alpha_fr-(4*pi/180)))).^2+(max(0,((-4*pi/180)-alpha_fr))).^2+(max(0,(alpha_rr-(4*pi/180)))).^2+(max(0,((-4*pi/180)-alpha_rr))).^2);
    alpha_cost=0.0;
    
    %four corners of the vehicle
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

    [obstacle_distances,lane_distances]=minDistance_tube(corners,alpha);
    
    obstacle_cost=sum(exp(-0.5*obstacle_distances));
    lane_cost=sum(exp(-5.0*lane_distances));

    for i=(1:N-1)
            u=U(i,:);
            if i==1
                deltaU=abs(u(1,1)-U0(1,1));
                stage_cost(i,1)=(w_U*(u(1,1))^2)+(w_sv*u(1,2))+(w_deltaU*(deltaU)^2);
            else
                deltaU=abs(u(1,1)-U(i-1,1));
                stage_cost(i,1)=(w_U*(u(1,1))^2)+(w_sv*u(1,2))+w_deltaU*(deltaU)^2;
            end
    end
    cost=sum(stage_cost,1)+w_alpha*alpha_cost+w_obstacle*obstacle_cost+w_lane*lane_cost;
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