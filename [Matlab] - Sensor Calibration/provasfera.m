clear all
close all
clc

fileResult = fopen ('result1.txt', 'r');
A = fscanf (fileResult, '%g %g %g', [3 inf]);    %ho il mio file in una matrice
fclose (fileResult);

A=A';
c1 = A(:, 1);
c2 = A(:, 2);
c3 = A(:, 3);

grid on
figure(1);
tri = delaunay(c1, c2);
h = trisurf(tri, c1, c2, c3);
shading interp;
axis vis3d
colormap (jet)


