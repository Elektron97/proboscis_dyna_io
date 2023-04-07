function se3 = vec6D2se3(vec6D)
    se3 = [skew(vec6D(1:3)), vec6D(4:6); zeros(1, 3), 0];
end