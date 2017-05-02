function out = GuantFinalWorks()
clear
% pub = rospublisher('/raw_vel');
% sub = rossubscriber('/encoders');
% msg = rosmessage(pub);
% sub_bump = rossubscriber('/bump');
hold on
Xwall = 3.378;
Ywall = 2.159;
b1xc = 0.8128;
b1yc = 0.8128;
b2xc = 1.5748;
b2yc = 1.3716;
b3xc = 1.9812;
b3yc = 0.1526;
side = 0.3048;


%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities
x = [0.01; 0.01];
grad = findGrad(x(1),x(2))
%gradient = double(subs(Gradient,[px; py], x))

% Define the function
% Test at the initial point
d = .25; % meters
%x = [4; 1]; % beginning point
phi = 0; % radians
lambda = 2; % step size
delta = .93; % change in step size
vel = 0.05; 
w = vel/(d/2); % angular velocity
vlr = -(w*d)/2; % left wheel speed (meters/second)
vrr = (w*d)/2; % right wheel speed (meters/second)
y = 0; % for while loop

%    bumpMessage = receive(sub_bump);
%     if any(bumpMessage.Data)
%         msg.Data=[0,0];
%         send(pub, msg);
%     end
while grad(1)< 3
%     bumpMessage = receive(sub_bump);
%     if any(bumpMessage.Data)
%         msg.Data=[0,0];
%         send(pub, msg);
%         break;
%     end
    %gradient = Gradient(x); % creates gradient from initial point
    
    shift = lambda.*grad; % multiplies step size by gradient
    leng = norm(shift) % gives length of resulting vector
    x = x + shift % shifted position (where the robot is)
    hold on;
    plot(x(1),x(2),'ro');
    axis([-.5 5 -.5 3])
    drawnow
    lambda = lambda*delta; % decreases step size a little
    theta = atan2(grad(2),grad(1)); % angle robot is currently at
    delta_phi = theta-phi; % pre-movement angle
    if delta_phi < -pi
       delta_phi = delta_phi + 2*pi; % makes sure the robot doesn't make a full rotation and instead makes a small rotation
   end


    movetime = leng/vel %m/m*s* (m/s *f/m== f/s /f)

    turn_time = delta_phi/w % how long it should take for the robot to turn
    if turn_time < 0 
        turn_time = -turn_time; % makes turn time positive for rotation
        Vrr = -vrr; % switches wheel velocities
        Vlr = - Vrr; % switches wheel velocities
    else 
        Vlr = vlr;
        Vrr = vrr;
    end
% 
%     msg.Data = [Vlr, Vrr];
%     send(pub, msg);
% 
%     pause(turn_time)
% 
%     msg.Data=[0,0];
%     send(pub, msg);
% 
%     vl = vel;
%     vr = vel;
%     msg.Data = [vl, vr];
% 
%     send(pub, msg);
%     pause(movetime)
%     
%     msg.Data=[0,0];
%     send(pub, msg);
%     pause(.5)

    phi = theta; % updates current angle
    grad = findGrad(x(1),x(2))
end    

% msg.Data = [0, 0];
% send(pub, msg);

function grad = findGrad(x1,y1)
    x1 =[x1,y1]
    [px,py]=meshgrid(x1(1)-0.5:.15:x1(1)+0.5,x1(2)-0.5:.15:x1(2)+0.5);
    [xlim,ylim] = size(px);
    V = zeros(xlim, ylim);
    Cboxes_walls = .7;
    Cbucket = -10;
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
    %%

    %%
    % Cpoint = -20;
    % point1 =  1./sqrt((px-(2.8194)).^2 + ((py-1.397)).^2)
    % V = Cpoint* point1
    V;
    [Ex, Ey] = gradient(V);
    Gradient = [-Ex(4,4); -Ey(4,4)];
    grad = Gradient;
end
out=2;
end


