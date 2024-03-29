function cost=costFunc(U0,U,rmpcProps)
    %{
    Calculates the cost over the horizon. The U is a matrix over the
    horizon with each row containing the u and epsilon values needed to
    calculate the cost. U0 is used to calculate the delta at the very first
    time step
    %}
    persistent n N w_deltaU w_U w_sv
    if isempty(n)
        n=0;
        N=rmpcProps.N;
        w_deltaU=rmpcProps.w_deltaU;
        w_U=rmpcProps.w_U;
        w_sv=rmpcProps.w_sv;
    else
        %do nothing
    end
    stage_cost=zeros(N,1);
    for i=(1:N-1)
            u=U(i,:);
            if i==1
                deltaU=u(1,1)-U0(1,1);
                stage_cost(i,1)=(w_U*(u(1,1))^2)+(w_sv*u(1,2))+(w_deltaU*(deltaU)^2);
            else
                deltaU=u(1,1)-U(i-1,1);
                stage_cost(i,1)=(w_U*(u(1,1))^2)+(w_sv*u(1,2))+w_deltaU*(deltaU)^2;
            end
    end
    cost=sum(stage_cost,1);
end