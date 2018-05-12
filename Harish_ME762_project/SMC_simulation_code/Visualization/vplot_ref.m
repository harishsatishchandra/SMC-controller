function TT=vplot_ref(bench)
%% trajectory
TT=bench.data.T;
XT=bench.data.tau.X(TT)';
YT=bench.data.tau.Y(TT)';

figure(1);
hold all;box on;
h1 = plot(XT,YT,'-.','Color','red','LineWidth',1.5);
%uistack(h1,'top')
xlabel('X (m)');
ylabel('Y (m)');
drawnow

end