function plot3vec(v, length) % assumes input is N x 3 matrix

if nargin == 1
    length = ones(size(v,1),1);
end

v = v.*repmat(length,1,3); % replicate theta vector into Nx3 matrix, and multiply
r = reshape(v,1,[]); % put data all in one row
r = [zeros(size(r));r]; % interleave 0's in between the data
r = reshape(r,[],3); % reshape back to 2N x 3
plot3(r(:,1),r(:,2),r(:,3),'-k.'); % plot it
grid on

end