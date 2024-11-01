function Lxi = FuncLx(x, y, Z, Lambda)
    Lxi = [-1/Z, 0, x/Z;
           0, -1/Z, y/Z];
    Lxi = Lambda * Lxi;
end
