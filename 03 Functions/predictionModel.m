function x=predictionModel(x0,driverProps,vehProps,rmpcProps,refTraj,t_final)
    %{
    x(1)=body slip angle
    x(2)=yaw rate
    x(3)=driver steering angle
    x(4)=heading angle
    x(5)=delta_y
    x(6)=X (global frame,CG)
    x(7)=Y (global frame,CG)
    %}
    persistent n time_step C1 C2 a b V CMz1 CMz2 m Jz Kd tau La
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
    tspan=(0:time_step:t_final)';
    x=zeros(length(tspan),length(x0));  %kx7 matrix
    x(1,:)=x0;                          %first row will be initial conditions
    %calculating the psi_ref needed to determine the driver steering angle
    for i = (1:length(tspan)-1)
        psi_ref = makima(refTraj.X,refTraj.psi_ref,(x(i,6)+La));
        dbeta_dt=[(Yb/(m*V)) ((Yr/(m*V))-1) (Yd/(m*V))]*[x(i,1) x(i,2) x(i,3)]';
        dr_dt=[(Nb/Jz) (Nr/Jz) (Nd/Jz)]*[x(i,1) x(i,2) x(i,3)]';
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