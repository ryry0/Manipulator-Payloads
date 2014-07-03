
function [mtot, lc2tot, I2tot] = AddLoad(m2, lc2, I2, ml, lcl, Il)
%This adds a load to the manipulator and correctly updates the
%center of mass, mass, and inertia.
mtot = m2 + ml;

lc2tot = (lc2*m2 + lcl*ml)/(mtot);

%reflect the inertias back to the beginning of the link
I2_reflected = I2 + m2*(lc2^2);
Il_reflected = Il + ml*(lcl^2);

I_reflected_total = I2_reflected + Il_reflected;

%reflect the inertias back to the updated center of mass
I2tot = I_reflected_total - mtot*(lc2tot^2);
end