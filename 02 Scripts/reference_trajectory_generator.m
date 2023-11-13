%Compute double lane change reference track: Cosine Y form
XL=70;% m, total distance
dx=0.1;
X=0:dx:XL;
for i=1:1:length(X)
    if X(i)<15
        Y(i)=0;
        phi_o(i)=0;
    elseif (X(i)>=15)&&(X(i)<45)
        Y(i)=3.5/2*(1-cos(pi/30*(X(i)-15)));
        phi_o(i)=atan(3.5*pi/60*sin(pi/30*(X(i)-15)));
    elseif (X(i)>=45)&&(X(i)<=70)
        phi_o(i)=0;
        Y(i)=3.5;
%     elseif (X(i)>=70)&&(X(i)<95)
%         phi_o(i)=-atan(3.5*pi/50*sin(pi/25*(X(i)-70)));
%         Y(i)=3.5/2*(1+cos(pi/25*(X(i)-70)));
%     elseif (X(i)>=95)&&(X(i)<=XL)
%         phi_o(i)=0;
%         Y(i)=0;
    end
end

refTraj.X=X';
refTraj.Y=Y';
refTraj.psi_ref=phi_o';