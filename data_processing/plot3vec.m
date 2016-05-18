function plot3vec(v, length, style) % assumes input is N x 3 matrix

% if length of vector is not given, set it to 1
if nargin == 1
    length = ones(size(v,1),1);
    style = '-k.';
elseif nargin == 2
    style = length;
    length = ones(size(v,1),1);
end

v = v.*repmat(length,1,3); % replicate theta vector into Nx3 matrix, and multiply
r = reshape(v,1,[]); % put data all in one row
r = [zeros(size(r));r]; % interleave 0's in between the data
r = reshape(r,[],3); % reshape back to 2N x 3
plot3(r(:,1),r(:,2),r(:,3),style); % plot it
grid on

end