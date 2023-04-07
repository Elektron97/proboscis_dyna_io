%%%%%%%% Biezer Curve %%%%%%%
function curve = biezerCurve(var, p0, p1, p2, p3)
curve = ((1- var)^3)*p0 + 3*var*((1 - var)^2)*p1 + ...
        3*(var^2)*(1-var)*p2 + (var^3)*p3;
end