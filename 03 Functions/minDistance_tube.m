%{
This function calculates the minimum distance between a point and a line
segment. This is currently being used to get the distance between a point
and an obstacle defined in the problem setup for the course project. Input
to this function is a 2xN matrix of points that represent the footprint of
the vehicle. Output is the minimum distance each of these points with the
lane line and the obstacle.
%}
function [min_obstacle_dist,min_lane_dist]=minDistance_tube(corners,alpha)

    persistent n o1_x1 o1_x2 o1_y1 o1_y2 o2_x1 o2_x2 o2_y1 o2_y2
    persistent l_x1 l_x2 l_y1 l_y2
    if isempty(n)
        n=0;
        %segments that define the sides of the obstacle
        o1_x1=35.0; 
        o1_x2=35.0; 
        o1_y1=-0.75; 
        o1_y2=1.75+(alpha*1.75); %original - 1.75m 
        o2_x1=35.0; 
        o2_x2=75.0; 
        o2_y1=1.75+(alpha*1.75); %original - 1.75m 
        o2_y2=1.75+(alpha*1.75); %original - 1.75m 
        %segment that define the lane line
        l_x1=-10.0; 
        l_x2=80.0; 
        l_y1=5.0-(alpha*5.0); %original - 5.0m 
        l_y2=5.0-(alpha*5.0); %original - 5.0m 
    else
        %do nothing
    end
    
    %extracting all the pairs over the horizon
    x=corners(1,:)';
    y=corners(2,:)';
    %calculating certain values needed to calculate the min distance
    %obstacle 1 - first segment
    A1=x-o1_x1;
    B1=y-o1_y1;
    C1=o1_x2-o1_x1;
    D1=o1_y2-o1_y1;
    dot_1=A1.*C1+B1.*D1;
    lnSq_1=C1.*C1+D1.*D1;
    param1=dot_1./lnSq_1;
    xx_1=(param1<0)*(o1_x1)+(param1>1)*(o1_x2)+((param1>0).*(param1<=1)).*(o1_x1+param1.*C1);
    yy_1=(param1<0)*(o1_y1)+(param1>1)*(o1_y2)+((param1>0).*(param1<=1)).*(o1_y1+param1.*D1);
    distances_1=sqrt((x-xx_1).^2+(y-yy_1).^2);
    %obstacle 1 - second segment
    A2=x-o2_x1;
    B2=y-o2_y1;
    C2=o2_x2-o2_x1;
    D2=o2_y2-o2_y1;
    dot_2=A2.*C2+B2.*D2;
    lnSq_2=C2.*C2+D2.*D2;
    param2=dot_2./lnSq_2;
    xx_2=(param2<0)*(o2_x1)+(param2>1)*(o2_x2)+((param2>0).*(param2<=1)).*(o2_x1+param2.*C2);
    yy_2=(param2<0)*(o2_y1)+(param2>1)*(o2_y2)+((param2>0).*(param2<=1)).*(o2_y1+param2.*D2);
    distances_2=sqrt((x-xx_2).^2+(y-yy_2).^2);
    %multiplication with temp to make sure distance becomes zero inside the
    %obstacle
    temp=((x>o1_x1).*(x<o2_x2)).*((y>o1_y1).*(y<o1_y2));
    distances=(1-temp).*[distances_1 distances_2];
    min_obstacle_dist=min(distances,[],2);
    %upper lane
    A3=x-l_x1;
    B3=y-l_y1;
    C3=l_x2-l_x1;
    D3=l_y2-l_y1;
    dot_3=A3.*C3+B3.*D3;
    lnSq_3=C3.*C3+D3.*D3;
    param3=dot_3./lnSq_3;
    xx_3=(param3<0)*(l_x1)+(param3>1)*(l_x2)+((param3>0).*(param3<=1)).*(l_x1+param3.*C3);
    yy_3=(param3<0)*(l_y1)+(param3>1)*(l_y2)+((param3>0).*(param3<=1)).*(l_y1+param3.*D3);
    temp=(y>l_y1);
    min_lane_dist=(1-temp).*(sqrt((x-xx_3).^2+(y-yy_3).^2));
end