%%%%%%%%%%% Pretty Strain Twist Plot %%%%%%%%%%
function prettyStrainPlot(xi)
xi_norm = xi/norm(xi);
% Bar Plot of Strain Modes
strain_modes = categorical({'Bending_x', 'Bending_y', 'Torsion_z', 'Shear_x', 'Shear_y', 'Stretch_z'});
strain_modes = reordercats(strain_modes, {'Bending_x', 'Bending_y', 'Torsion_z', 'Shear_x', 'Shear_y', 'Stretch_z'});

figure
bar(strain_modes, xi_norm');
grid on
ylabel("Strain Deformation");
title("Strain Modes Excited");
end