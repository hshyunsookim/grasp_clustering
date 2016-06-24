%% Pre-Processing
clc, clear

addpath('../data_logs')
% M = dlmread('datalog_20160602-131519.txt');
M = dlmread('datalog_20160602-142723.txt');
% M = dlmread('datalog_20160607-115351.txt');

xform = M(:,1:12);
quat = zeros(size(M,1),4);
for i=1:size(M,1)
    A = reshape(xform(i,1:9),3,3);
    quat(i,:) = rotm2quat(A);

    [V,D] = eig(A);
    [ignore,ix] = min(abs(diag(D)-1));
    w(i,:) = V(:,ix)';
    t = [A(3,2)-A(2,3),A(1,3)-A(3,1),A(2,1)-A(1,2)];
    theta(i,1) = atan2(t*w(i,:)',trace(A)-1);
    if theta(i,1)<0, theta(i,1) = -theta(i,1); w(i,:) = -w(i,:); end
end

x = xform(:,10);
y = xform(:,11);
z = xform(:,12);

config = M(:,13:end-1);
swivel_finger = config(:,[1,3]);
proximal_finger = config(:,[2,4,5]);

label = M(:,end);

clearvars -except M xform quat w theta x y z config proximal_finger swivel_finger label
% M = [w,theta,x,y,z,label];
M = [quat,x,y,z,label];
save('M','M');

%% Plot Settings
grasp_types = 2;
samples = [size(M,1)];
styles = {'k.', 'r.', 'k.'};

%% 
pos=label==1;
neg=label==0;
%% Plot
i=1;
figure(1)
subplot(2,2,1)
plot3vec(w(pos,:), theta(pos),  styles{i}), hold on
% axis([-2 5 -2 5 -2 5])
axis equal
xlabel('x'), ylabel('y'), zlabel('z')
title('rotation axis * angle')

subplot(2,2,2)
plot3vec([x(pos,:),y(pos,:),z(pos,:)], styles{i}), hold on
% axis([-0.2 0.2 -0.2 0.2 -0.2 0.2])
axis equal
xlabel('x'), ylabel('y'), zlabel('z')
title('relative position')

subplot(2,2,3)
plot3(proximal_finger(pos,1),proximal_finger(pos,2),proximal_finger(pos,3),styles{i}), hold on
% axis([0 2 0 2 0 2])
title('finger flexion')

subplot(2,2,4)
plot(swivel_finger(pos,1), swivel_finger(pos,2), styles{i}), hold on
% axis([-1 1 -1 1])
title('finger swivel')

i=2;
figure(1)
subplot(2,2,1)
plot3vec(w(neg,:), theta(neg),  styles{i}), hold on
% axis([-2 5 -2 5 -2 5])
axis equal
xlabel('x'), ylabel('y'), zlabel('z')
title('rotation axis * angle')

subplot(2,2,2)
plot3vec([x(neg,:),y(neg,:),z(neg,:)], styles{i}), hold on
% axis([-0.2 0.2 -0.2 0.2 -0.2 0.2])
axis equal
xlabel('x'), ylabel('y'), zlabel('z')
title('relative position')

subplot(2,2,3)
plot3(proximal_finger(neg,1),proximal_finger(neg,2),proximal_finger(neg,3),styles{i}), hold on
% axis([0 2 0 2 0 2])
title('finger flexion')

subplot(2,2,4)
plot(swivel_finger(neg,1), swivel_finger(neg,2), styles{i}), hold on
% axis([-1 1 -1 1])
title('finger swivel')

%%
figure(2)
subplot(1,2,1)
plot3(quat(pos,1), quat(pos,2), quat(pos,3), 'b.'), hold on
plot3(quat(neg,1), quat(neg,2), quat(neg,3), 'r.')
axis equal
xlabel('q1'), ylabel('q2'), zlabel('q3')
title('quaternion (qx,qy,qz)')

subplot(1,2,2)
plot3(w(pos,1), w(pos,2), w(pos,3), 'b.'), hold on
plot3(w(neg,1), w(neg,2), w(neg,3), 'r.')
axis equal
xlabel('q1'), ylabel('q2'), zlabel('q3')
title('quaternion (qx,qy,qz)')