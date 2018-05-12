function [  ] = visualize( benchmark )
%VISUALIZE
close all

steps=size(benchmark.data.T,1);
tspan=benchmark.data.T;
curve(1,:)=benchmark.data.tau.X(tspan)';
curve(2,:)=benchmark.data.tau.Y(tspan)';
curveD(1,:)=benchmark.data.tauD.X(tspan)';
curveD(2,:)=benchmark.data.tauD.Y(tspan)';
Y=benchmark.data.X;
d=2;
DISPLAY_AXIS=[min(curve(1,:))-d max(curve(1,:))+d min(curve(2,:))-d max(curve(2,:))+d];%benchmark.data.tau.DISPLAY_AXIS;
ts=mean(diff(benchmark.data.T));
dyn=CARparameters;

% %setup video
% vidObj = VideoWriter([benchmark.controller],'Archival');
% %vidObj.Quality = 100;
% vidObj.FrameRate = 30;
% open(vidObj);
% x0=10;
% y0=10;
% width=800;
% height=600;
% set(gcf,'units','points','position',[x0,y0,width,height])
figure('units','normalized','position',[.2 .4 .6 .4])

for k=1:steps
    tim=tic;
    
    %Y 1,2 is center of mass
    x=Y(k,1);
    y=Y(k,2);
    theta=Y(k,3);
    delta=benchmark.data.U(k,1);
    
    front=[x+dyn.l_F*cos(theta),y+dyn.l_F*sin(theta)];
    rear=[x-dyn.l_R*cos(theta),y-dyn.l_R*sin(theta)];
    wheel=[0.5*cos(theta+delta);0.5*sin(theta+delta)];
    heading=front-rear;
    %ref=get_trajectory(benchmark.data.path,t);
    
    plot(curve(1,:),curve(2,:),'r-','LineWidth',1);
    hold on
    plot(curveD(1,:),curveD(2,:),'b-.','LineWidth',1);
    axis(DISPLAY_AXIS);
    hold on
    %plot XY trajectory
    plot(Y(:,1),Y(:,2),'k-','LineWidth',1);
    %plot traj of a look-ahead wrt vehicle
    look_ahead=0;
    if strcmp(benchmark.data.control_point,'CO')
        look_ahead=dyn.J/dyn.m/dyn.l_R + dyn.l_R*0;
    else 
        if strcmp(benchmark.data.control_point,'REAR')   
            look_ahead=-dyn.l_R;
        end
    end
    plot(Y(:,1)+look_ahead.*cos(Y(:,3)), Y(:,2)+look_ahead.*sin(Y(:,3)),'g-.','LineWidth',0.5);
    %plot(Y(:,1)+cos(Y(:,3)).*0.5,Y(:,2)+sin(Y(:,3)).*0.5);
    plot([front(1),front(1)+wheel(1)],[front(2),front(2)+wheel(2)],'LineWidth',2);
    plot([front(1),front(1)-wheel(1)],[front(2),front(2)-wheel(2)],'LineWidth',2);
    quiver(rear(1),rear(2),heading(1),heading(2),'LineWidth',3,'MaxHeadSize', 2/norm(heading));
    plot(x,y,'.k', 'MarkerSize',25);
    plot(x,y,'.w', 'MarkerSize',20);
    
    %plot(ref(1),ref(2),'xr');
    %axis equal
    drawnow limitrate
    %     writeVideo(vidObj, getframe(gca));
    
    hold off
    %semirt
    dtoc=toc(tim);
    while dtoc<=ts
        dtoc=toc(tim);
    end
end
%
% close(gcf)
% close(vidObj)

% plot(curve(1,:),curve(2,:),'LineWidth',1);
% axis(DISPLAY_AXIS);
% hold on
% plot(Y(:,1),Y(:,2),'--','LineWidth',1.5);
% legend \tau_d HCO
end
