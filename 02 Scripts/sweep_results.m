%load mat files
matfiles=dir('*.mat');

for m = (1:length(matfiles))
    load(matfiles(m).name);
    figure(3)
    plot(mpc_control.X,mpc_control.Y,'k-');
    hold on
    plot(mpc_control_tube.X_a,mpc_control_tube.Y_a,'r-');
% 
    footprint_x=[mpc_control_tube.footprint_a(:,1);mpc_control_tube.footprint_a(:,2);mpc_control_tube.footprint_a(:,4);mpc_control_tube.footprint_a(:,3)];
    footprint_y=[mpc_control_tube.footprint_a(:,5);mpc_control_tube.footprint_a(:,6);mpc_control_tube.footprint_a(:,8);mpc_control_tube.footprint_a(:,7)];

    violation(m,1)=(sum((footprint_x>=35.0).*(footprint_x<=75.0).*(footprint_y>=-0.75).*(footprint_y<=1.75)))>0;
    
%     figure(2)
%     plot(mpc_control.u_ctrl,'k-');
%     plot(mpc_control_tube.v,'ro-');
%     hold on
%     for i = (1:2:length(mpc_control.footprint))
% %         patch([mpc_control.footprint(i,1) mpc_control.footprint(i,2) mpc_control.footprint(i,4) mpc_control.footprint(i,3)],[mpc_control.footprint(i,5) mpc_control.footprint(i,6) mpc_control.footprint(i,8) mpc_control.footprint(i,7)],'black');
%         patch([mpc_control_tube.footprint_a(i,1) mpc_control_tube.footprint_a(i,2) mpc_control_tube.footprint_a(i,4) mpc_control_tube.footprint_a(i,3)],[mpc_control_tube.footprint_a(i,5) mpc_control_tube.footprint_a(i,6) mpc_control_tube.footprint_a(i,8) mpc_control_tube.footprint_a(i,7)],'red');
%         alpha(0.2);
%     end
end
violations=length(nonzeros(violation));
%% analysing the control inputs
figure(3)
plot(mpc_control_tube.delta_dr_a,'ro-');
hold on
plot(mpc_control_tube.v,'ko-');
    