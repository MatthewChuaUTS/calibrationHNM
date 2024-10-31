function [Lx] = FuncLx(x,y,Z, lambda)
    Lx = zeros(2,6);
    
    Lx(1,1) = -lambda/Z;
    Lx(1,2) = 0;
    Lx(1,3) = x/Z;
    Lx(1,4) = x*y/lambda;
    Lx(1,5) = -(lambda^2+x^2)/lambda;
    Lx(1,6) = y;
    
    Lx(2,1) = 0;
    Lx(2,2) = -lambda/Z;
    Lx(2,3) = y/Z;
    Lx(2,4) = (lambda^2+y^2)/lambda;
    Lx(2,5) = -(x*y)/lambda;
    Lx(2,6) = -x;
end