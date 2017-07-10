plot3(x,y,z,'.-')

tri = delaunay(x,y);
plot(x,y,'.')

[r,c] = size(tri);
disp(r)
h = trisurf(tri, x, y, z);
axis vis3d

l = light('Position',[-50 -15 29])
%set(gca,'CameraPosition',[208 -50 7687])
lighting phong
shading interp
colorbar EastOutside
