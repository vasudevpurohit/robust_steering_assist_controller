%{
This function is a generalized formulation for an MPC with the optimization
carried out using fmincon
%}
function [u_ctrl,sv] = steerAssistController(x0,U0,rmpcProps,vehProps,driverProps,refTraj)
    %extracting the controller settings
    Aieq=rmpcProps.Aieq;
    Bieq=rmpcProps.Bieq;
    Aeq=rmpcProps.Aeq;
    Beq=rmpcProps.Beq;
    lb=rmpcProps.lb;
    ub=rmpcProps.ub;
    %defining the options for the fmincon solver
    options = optimoptions('fmincon','Display','off','Algorithm','interior-point','MaxIterations',50,'MaxFunctionEvaluations',1e5);
    %main fmincon call
    %{
    x0(1,1)=beta
    x0(2,1)=r
    x0(3,1)=delta
    x0(4,1)=psi
    x0(5,1)=delta_y;
    x0(6,1)=X;  (global frame)
    x0(7,1)=Y;  (global frame)
    %}
    U = fmincon(@(U)costFunc(U0,U,rmpcProps),U0,Aieq,Bieq,Aeq,Beq,lb,ub,@(U)safetyConstraints(x0,U,driverProps,vehProps,rmpcProps,refTraj),options);
    %U=u;sv over the horizon
    u_ctrl=U(1,1);
    sv=U(1,2);
end