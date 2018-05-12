function y = fwrap(f, x)
if size(x,2) == 1
    y = f(x);
else
    y = arrayfun(f, x, 'uni', 0);
    y = cat(2, y{:});
end