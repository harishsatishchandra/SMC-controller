function Y=fval(pp,x_int)

br = pp.breaks.';
cf = pp.coefs;

[~, inds] = histc(x_int, [-inf; br(2:end-1); +inf]);

x_shf = x_int - br(inds);
zero  = ones(size(x_shf));
one   = x_shf;
two   = one .* x_shf;
three = two .* x_shf;

Y = sum( [three two one zero] .* cf(inds,:), 2);