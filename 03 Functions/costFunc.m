function cost=costFunc(U0,U,N,w_deltaU,w_U,w_sv)
    %{
    Calculates the cost over the horizon. The U is a matrix over the
    horizon with each row containing the u and epsilon values needed to
    calculate the cost. U0 is used to calculate the delta at the very first
    time step
    %}
    stage_cost=zeros(N,1);
    for i=(1:N-1)
            u=U(i,:)';
            if i==1
                deltaU=u-U0(1,1);
                stage_cost(i,1)=(u(i,1)'*w_U*u(i,1))+(w_sv*u(i,2))+(deltaU'*w_deltaU*deltaU);
            else
                deltaU=u-U(i-1,1);
                stage_cost(i,1)=(u(i,1)'*w_U*u(i,1))+(w_sv*u(i,2))+deltaU'*w_deltaU*deltaU;
            end
    end
    cost=sum(stage_cost,1);
end