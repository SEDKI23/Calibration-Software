function roi = helperComputeROIOuster_vorne(imageCorners3d, tolerance)
%helperComputeROI computes ROI in lidar coordinate system using
%   checkerboard 3d corners in camera coordinate system
%
% This is an example helper function that is subject to change or removal in
% future releases.

% Copyright 2019-2020 The MathWorks, Inc.


xCamera = reshape(imageCorners3d(:, 1, :), [], 1);
yCamera = reshape(imageCorners3d(:, 2, :), [], 1);
zCamera = reshape(imageCorners3d(:, 3, :), [], 1);

xMaxLidar = min(zCamera) - tolerance;
xMinLidar = - (max(zCamera) + tolerance);

yMaxLidar = max(xCamera) + tolerance;
yMinLidar = min(xCamera) - tolerance;

zMaxLidar = max(yCamera) + tolerance;
zMinLidar = min(yCamera) - tolerance;

roi = [xMinLidar, xMaxLidar, yMinLidar, yMaxLidar, zMinLidar, zMaxLidar];
end