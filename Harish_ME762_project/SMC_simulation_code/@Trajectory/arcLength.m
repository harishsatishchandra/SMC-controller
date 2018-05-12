function [S,W] = arcLength(X,Y)
% X: tau->X, tau in [0,1]
% Y: tau->Y, tau in [0,1]
% S: tau->int(sqrt(dX^2+dY^2))
% W: S^-1
dTauR = 0.05;
nTauF = 5;
taunum_rough = 0:dTauR:1;
taunum_fine = linspace(0,dTauR,nTauF);

xj = X(taunum_rough(1));
yj = Y(taunum_rough(1));
snum = zeros(size(taunum_rough));
snum(1) = 0;
Si_num = zeros(size(taunum_fine));
gi = zeros(length(taunum_fine),3);
% P = zeros(size(gi,1),length(taunum_fine)-1);
Q = cell(length(taunum_rough)-1);

for i = 1:length(taunum_rough)-1
    Si_num(1) = snum(i);
    gi(1,3) = taunum_rough(i);
    gi(1,2) = snum(i);
    for k = 2:length(taunum_fine);
        xk = X(taunum_rough(i)+taunum_fine(k));
        yk = Y(taunum_rough(i)+taunum_fine(k));
        dist = sqrt((xk-xj).^2+(yk-yj).^2);
        Si_num(k) = Si_num(k-1) + dist;
        gi(k,3) = taunum_rough(i) + taunum_fine(k);
        gi(k,2) = Si_num(k);
        xj = xk;
        yj = yk;
    end
    Q{i} = create_poly(create_poly_coefficients(gi));
    snum(i+1) = Si_num(end);
end
S = @(tau)interp1(taunum_rough,snum,tau);
W = @(s)interpWn(snum,Q,s);
% clf; hold all;
% s = S(0):0.01:S(1);
% plot(s,W(s));
% plot(s(1:end-1),diff(W(s)));
% asdf = 0;



function tau = interpWn(snum,Q,s)
tau = zeros(size(s));
k = 1;
for i = 1:length(s)
    %search for the index / Definitionsbereich des Polynoms
    while(1)
        if snum(k)>s(i)
            k = k-1;
            if k<1
                k = 1;
                break;
            end
        else if snum(k+1)<s(i)
                k = k+1;
                if k+1>length(snum)
                    k = length(snum)-1;
                    break;
                end
            else
                break;
            end
        end
    end
    tau(i) = Q{k}(s(i));
end



