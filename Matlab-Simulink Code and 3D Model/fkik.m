positions = [0.03    0.0378   -0.0900];
linkLengths = [0.04914;0.083;0.095];
dhparam = [[0;0;0],linkLengths,[90;0;0]];

l1 = linkLengths(1);
l2 = linkLengths(2);
l3 = linkLengths(3);
P = positions';
P0 = zeros(3,1);

    
    p = P(1);
    q = P(2);
    t = sqrt(p^2 + q^2);
    if t == 0
        a1 = 0;
    else
        if q >= 0
            a1 = acosd(p/t);
        else
            a1 = -acosd(p/t);
        end
    end
    
    P1 =  l1 * [cosd(a1);sind(a1 ); 0];
    
    u = P - P1;
    r = sqrt(u(1)^2 + u(2)^2);
    s = sqrt(u(1)^2 + u(2)^2 + u(3)^2);
    
    v = (l2^2 + s^2 - l3^2) / (2 * l2 * s);
    
    if u(3) < 0
        a2 = acosd(v) - acosd(r/s);
    else
        a2 = acosd(v) + acosd(r/s);
    end
    
    a3 = acosd((l2^2 + l3^2 - s^2) / (2 * l2 * l3))-180;

a1 = 34.6703;
a2 = -71.5857;
a3 = 53.435;

P1 = l1 * [cosd(a1); sind(a1); 0];
P2 = P1 + l2 * [ cosd(a1)*cosd(a2); sind(a1)*cosd(a2) ; sind(a2)];
P3 = P2 + l3 * [cosd(a1)*cosd(a2+a3) ;sind(a1)*cosd(a2+a3)  ; sind(a2+a3)];

display([positions;P3']);

