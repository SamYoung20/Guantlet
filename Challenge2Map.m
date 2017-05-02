Xwall = 3.378;
Ywall = 2.159;
b1xc = 0.8128;
b1yc = 0.8128;
b2xc = 1.5748;
b2yc = 1.3716;
b3xc = 1.9812;
b3yc = 0.1526;
side = 0.3048;

clf
%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

[px,py]=meshgrid(-1:.15:ceil(Xwall)+1,-1:.15:ceil(Ywall)+1);
[xlim,ylim] = size(px);
V = zeros(xlim, ylim);
Cboxes_walls = .7
Cbucket = -10
for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want

%Walls of Pen
 wall1dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)+.5).^2));
 wall2dy = @(y)  1./sqrt(((px(i,j)-Xwall-.5).^2) + ((py(i,j)-y).^2));
 wall3dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- Ywall-.5).^2));
 wall4dy = @(y)  1./sqrt(((px(i,j)+ 0.5).^2) + ((py(i,j)-y).^2));
 wall1 = integral(wall1dx,0,Xwall);
 wall2 = integral(wall2dy,0,Ywall);
 wall3 = integral(wall3dx,0,Xwall);
 wall4 = integral(wall4dy,0,Ywall);
 walls = (Cboxes_walls* wall1) + (Cboxes_walls* wall2) + (Cboxes_walls* wall3) +(Cboxes_walls* wall4);
 
 %
 %Bucket
 point = 1./sqrt(((px(i,j)-(2.8194)).^2 )+ ((py(i,j)-1.397)).^2);
 %
 % Box 1
 b1s1dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- b1yc).^2));
 b1s2dy = @(y)  1./sqrt(((px(i,j)-b1xc-side).^2) + ((py(i,j)-y).^2));
 b1s3dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- b1yc-side).^2));
 b1s4dy = @(y)  1./sqrt(((px(i,j)-b1xc).^2) + ((py(i,j)-y).^2));
 b1s1 = integral( b1s1dx,b1xc,(b1xc+side));
 b1s2 = integral(b1s2dy,b1yc,(b1yc+side));
 b1s3 = integral(b1s3dx,b1xc,(b1xc+side));
 b1s4 = integral(b1s4dy,b1yc,b1yc+side);
 box1 = (Cboxes_walls* b1s1) + (Cboxes_walls* b1s2)+ (Cboxes_walls* b1s3)+ (Cboxes_walls* b1s4);
 
%
%Box2
b2s1dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- b2yc).^2));
b2s2dy = @(y)  1./sqrt(((px(i,j)-b2xc-side).^2) + ((py(i,j)-y).^2));
b2s3dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- b2yc-side).^2));
b2s4dy = @(y)  1./sqrt(((px(i,j)-b2xc).^2) + ((py(i,j)-y).^2));
b2s1 = integral( b2s1dx,b2xc,(b2xc+side));
b2s2 = integral(b2s2dy,b2yc,(b2yc+side));
b2s3 = integral(b2s3dx,b2xc,(b2xc+side));
b2s4 = integral(b2s4dy,b2yc,b2yc+side);
box2 = (Cboxes_walls* b2s1) + (Cboxes_walls* b2s2)+ (Cboxes_walls* b2s3)+ (Cboxes_walls* b2s4);
%
%Box3
b3s1dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- b3yc).^2));
b3s2dy = @(y)  1./sqrt(((px(i,j)-b3xc-side).^2) + ((py(i,j)-y).^2));
b3s3dx = @(x)  1./sqrt(((px(i,j)-x).^2) + ((py(i,j)- b3yc-side).^2));
b3s4dy = @(y)  1./sqrt(((px(i,j)-b3xc).^2) + ((py(i,j)-y).^2));
b3s1 = integral( b3s1dx,b3xc,(b3xc+side));
b3s2 = integral(b3s2dy,b3yc,(b3yc+side));
b3s3 = integral(b3s3dx,b3xc,(b3xc+side));
b3s4 = integral(b3s4dy,b3yc,b3yc+side);
box3 = (Cboxes_walls* b3s1) + (Cboxes_walls* b3s2)+ (Cboxes_walls* b3s3)+ (Cboxes_walls* b3s4);
%
 V(i,j) = box1 + box2 + box3 + walls + Cbucket*point;
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
title('Challenge 2')
plot([0 0 Xwall Xwall 0],[0 Ywall Ywall 0 0] ,'b-')
hold on
plot([b1xc (b1xc+side) (b1xc+side) b1xc b1xc],[b1yc b1yc (b1yc+side) (b1yc+side) b1yc] ,'b-')
hold on
plot([b2xc (b2xc+side) (b2xc+side) b2xc b2xc],[b2yc b2yc (b2yc+side) (b2yc+side) b2yc] ,'b-')
hold on
plot([b3xc (b3xc+side) (b3xc+side) b3xc b3xc],[b3yc b3yc (b3yc+side) (b3yc+side) b3yc] ,'b-')



