close all
[x,y,z] = sphere(50);
% colormap(jet)

subplot(1,2,1);
surf(x,y,z),shading interp
light('position',[2,-2,2],'style','local')
lighting phong
title('phong')

subplot(1,2,2)
surf(x,y,z,-z),shading flat
lighting flat
light('position',[-1,0.5,1],'style','local','color','w')
title('flat')
