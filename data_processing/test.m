%% Pre-Processing
clc, clear

addpath('../data_logs')
M = dlmread('datalog_20160518-151529.txt');

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
    if theta<0, theta = -theta; w = -w; end
end

x = xform(:,10);
y = xform(:,11);
z = xform(:,12);

config = M(:,13:end);
swivel_finger = config(:,[1,3]);
proximal_finger = config(:,[2,4,5]);

clearvars -except M xform quat w theta x y z config proximal_finger swivel_finger

%% Plot
figure(1), clf
subplot(2,2,1)
plot3vec(w(1:213,:), theta(1:213,:), '-k.'), hold on
plot3vec(w(214:end,:), theta(214:end,:), '-r.')
title('rotation axis * angle')

subplot(2,2,2)
plot3vec([x(1:213,:),y(1:213,:),z(1:213,:)], '-k.'), hold on
plot3vec([x(214:end,:),y(214:end,:),z(214:end,:)], '-r.')
title('relative position')

subplot(2,2,3)
plot3(proximal_finger(1:213,1),proximal_finger(1:213,2),proximal_finger(1:213,3),'k.'), hold on
plot3(proximal_finger(214:end,1),proximal_finger(214:end,2),proximal_finger(214:end,3),'r.')
axis([0 2 0 2 0 2])
title('finger flexion')

subplot(2,2,4)
plot(swivel_finger(:,1), swivel_finger(:,2), 'k.')
axis([-1 1 -1 1])
title('finger swivel')