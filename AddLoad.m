%%
% mtot is the total mass of the manipulator (load + link)
% lc2tot is the center of mass of the whole system
% I2tot is the total inertia of the system
% you have to give it the  mass, center of mass, and inertia of the base
% link
% Then you give it the mass of the load, the distance to the center of mass
% relative to the termination point of the link. Then, the inertia of the
% load.
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