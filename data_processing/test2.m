%% Pre-Processing
clc, clear

addpath('../data_logs')
% M = dlmread('datalog_20160519-144019.txt');
% M = dlmread('datalog_20160519-152020.txt');
% M = dlmread('datalog_20160519-153232.txt');
% M = dlmread('datalog_20160519-161304.txt');
% M = dlmread('datalog_20160520-114414.txt');
% M = dlmread('datalog_20160520-121153.txt');
M = dlmread('datalog_20160520-141826.txt');

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

config = M(:,13:end);
swivel_finger = config(:,[1,3]);
proximal_finger = config(:,[2,4,5]);

clearvars -except M xform quat w theta x y z config proximal_finger swivel_finger

%% Plot Settings
grasp_types = 3;
samples = [150, 230, size(M,1)];
styles = {'k.', 'b.', 'k.'};

%% Plot
start = 1;
for i=1:grasp_types
    figure(1)
    subplot(2,2,1)
    plot3vec(w(start:samples(i),:), theta(start:samples(i)),  styles{i}), hold on
    % axis([-2 5 -2 5 -2 5])
    axis equal
    xlabel('x'), ylabel('y'), zlabel('z')
    title('rotation axis * angle')

    subplot(2,2,2)
    plot3vec([x(start:samples(i),:),y(start:samples(i),:),z(start:samples(i),:)], styles{i}), hold on
    % axis([-0.2 0.2 -0.2 0.2 -0.2 0.2])
    axis equal
    xlabel('x'), ylabel('y'), zlabel('z')
    title('relative position')

    subplot(2,2,3)
    plot3(proximal_finger(start:samples(i),1),proximal_finger(start:samples(i),2),proximal_finger(start:samples(i),3),styles{i}), hold on
    axis([0 2 0 2 0 2])
    title('finger flexion')

    subplot(2,2,4)
    plot(swivel_finger(start:samples(i),1), swivel_finger(start:samples(i),2), styles{i}), hold on
    axis([-1 1 -1 1])
    title('finger swivel')
    
    start = samples(i)+1;
end