function vplot( bench,filename)
%% trajectory
TT=bench.data.T;
XT=bench.data.tau.X(TT)';
YT=bench.data.tau.Y(TT)';
XCGT=bench.data.X(:,1);
YCGT=bench.data.X(:,2);
PsiT=bench.data.X(:,3);

figure(1);clf;hold all;box on;
set(gcf(),'Name',[filename '- trajectory']);
h1 = plot(XT,YT,'Color','red');
h2 = plot(XCGT,YCGT,'Color','blue');
myquiver(TT,XCGT,YCGT,PsiT,20,'blue');
xlabel('X (m)');
ylabel('Y (m)');
legend([h1,h2],'[X,Y]','[X^{CG},Y^{CG}]','Location','NorthWest');
myprint([filename 'trajectory']);
drawnow

end

function myprint(filename)
set(gcf,'PaperPositionMode','auto');
saveas(gcf(),[filename '.fig']);
%print([filename '.png'],'-dpng','-r600');    
%print([filename '.eps'],'-deps2','-r300');
end
