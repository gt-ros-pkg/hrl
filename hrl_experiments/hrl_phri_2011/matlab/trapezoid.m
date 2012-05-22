function [y] = trapezoid(x,xdata)
    xdata_len = length(xdata);
    y = ones(1, xdata_len);
    if x(2) < x(1) 
        y = y * 0;
        return
    end
    y(xdata<=x(1)) = x(3)/x(1)*xdata(xdata<=x(1)) + 0.5;
    y(xdata>x(1) & xdata<=x(2)) = x(3) + 0.5;
    y(xdata>x(2)) = x(3)*(1-(x(2)-xdata(xdata>x(2)))/(x(2)-1)) + 0.5;
end
