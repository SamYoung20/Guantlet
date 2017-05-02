
Xwall = 3.378
Ywall = 2.159


%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

[px,py]=meshgrid(-1:.15:ceil(Xwall)+1,-1:.15:ceil(Ywall)+1);
[xlim,ylim] = size(px);
V = zeros(xlim, ylim);
Cboxes_walls = 0.1
Cbucket = -1
for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want

%right hand wall X
 wall1dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)+.5).^2));
 wall2dy = @(y)  1./sqrt(((px(i,j)-Xwall-.5).^2) + ((py(i,j)-y).^2));
 wall3dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- Ywall-.5).^2));
 wall4dy = @(y)  1./sqrt(((px(i,j)+ 0.5).^2) + ((py(i,j)-y).^2));
 wall1 = integral(wall1dx,0,Xwall);
 wall2 = integral(wall2dy,0,Ywall);
 wall3 = integral(wall3dx,0,Xwall);
 wall4 = integral(wall4dy,0,Ywall);
 point = 1./sqrt(((px(i,j)-(2.8194)).^2 )+ ((py(i,j)-1.397)).^2);
 V(i,j) = (Cboxes_walls* wall1) + (Cboxes_walls* wall2)+ (Cboxes_walls* wall3)+ (Cboxes_walls* wall4) + Cbucket*point;
    end
end
hold off
contour(px,py,V)
[Ex,Ey] = gradient(V);
hold on
streamslice(px,py,-Ex,-Ey)
hold on
plot(2.8194, 1.397,'rx')
hold on
title('Challenge 1')
plot([0 0 Xwall Xwall 0],[0 Ywall Ywall 0 0] ,'b-')

