clear all
close all
clc

filePreCalibrazione = fopen ('result.txt', 'r');
A = fscanf (filePreCalibrazione, '%g %g %g', [3 inf]);    %ho il mio file in una matrice
fclose (filePreCalibrazione);
A=A';
c1 = A(:, 1);
c2 = A(:, 2);
c3 = A(:, 3);

filePostCalibration = fopen ('M2POSTCALIBRAZIONE.txt', 'r');
B = fscanf (filePostCalibration, '%g %g %g', [3 inf]);    %ho il mio file in una matrice
fclose (filePostCalibration);
B=B';
c4 = B(:, 1);
c5 = B(:, 2);
c6 = B(:, 3);


[xg, yg, zg] = ellipsoid(0,0,0,37000, 37000, 37000, 100);
surfl(xg,yg,zg);
title(['\fontsize{20}{\color{red}PRECALIBRAZIONE}'])
colormap white;
alpha (.40000)
view([0.1, 0.1, 0.1])
axis on
axis equal   
axis square
axis([0 70000 0 70000 0 70000])
grid on
hold on
plot3 (c1, c2, c3, '.', 'MarkerSize', 4.8)

figure(2)
[xg2, yg2, zg2] = ellipsoid(0,0,0,47000, 47000, 47000, 100);
surfl(xg2,yg2,zg2); 
title(['\fontsize{20}{\color{green}POSTCALIBRAZIONE}'])
colormap white;
alpha (.40000)
view([0.1, 0.1, 0.1])
axis on
axis equal   
axis square
axis([0 70000 0 70000 0 70000])
grid on
hold on
plot3 (c4, c5, c6, '.', 'MarkerSize', 4.8)



