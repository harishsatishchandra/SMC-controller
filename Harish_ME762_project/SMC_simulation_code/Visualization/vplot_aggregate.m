function vplot_aggregate(bench,c)
%% trajectory
TT=bench.data.T;
XCGT=bench.data.X(:,1);
YCGT=bench.data.X(:,2);
PsiT=bench.data.X(:,3);

figure(1);
hold all;box on;
h2 = plot(XCGT,YCGT,'-','Color',c,'LineWidth',1.2);

%myquiver(TT,XCGT,YCGT,PsiT,20,h2.Color);

drawnow

end