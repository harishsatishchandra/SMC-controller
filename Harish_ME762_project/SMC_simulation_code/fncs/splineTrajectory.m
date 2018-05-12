function [ k1,vk1,ak1,jk1] = splineTrajectory(path,time)
%SPLINE position as function of time
% numerical approach
    [s,ds,dds,ddds]=path.s_profile.pva(time);
    %find segment
    segment=find(s<=path.cumarc,1);
    if s>path.cumarc(end)
        segment=size(path.cumarc,2);
    end
    %translate offset
    if segment>1
        s=s-(path.cumarc(segment-1));
    end
    dfpds=[path.dfp(segment,s);path.dfp(segment,s)];
    ddfpds=[path.ddfp(segment,s);path.ddfp(segment,s)];
    dddfpds=[path.dddfp(segment,s);path.dddfp(segment,s)];
    [k1,df1dfp,d2f1dfp,d3f1dfp]=spline2D(path.fp(segment,s),path.spl5{segment});
    %chain rule differentiation
    %Faà di Bruno's formula
    dg=df1dfp.*dfpds;
    ddg=d2f1dfp.*dfpds.^2+df1dfp.*ddfpds;
    dddg=d3f1dfp.*dfpds.^3+3*d2f1dfp.*dfpds.*ddfpds+df1dfp.*dddfpds;
    vk1=dg.*ds;
    ak1=ddg.*ds^2+dg.*dds;
    jk1=dddg.*ds.^3+3*ddg.*ds.*dds+dg.*ddds;
end

