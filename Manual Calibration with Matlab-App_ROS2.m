%Retrieve information from the bag file ROS2
path = "/home/bensalem/Documents/new_data/rosbag2_2024_01_15-14_47_34";
bagReader = ros2bagreader(path); 


% Select image and point cloud messages from the rosbag and select a subset of messages from the file by using the appropriate topic names.
imageBag = select(bagReader,'Topic','/drivers/zed_camera/right/driver/right/image_rect_color') ;
pcBag = select(bagReader,'Topic','/drivers/ouster_lidar/points') ;
CamerainfoBag = select(bagReader,'Topic','/drivers/zed_camera/right/driver/right/image_rect_color') ;

% Read all the messages.
imageMsgs = readMessages(imageBag) ;
pcMsgs = readMessages(pcBag) ;

% Intrinsc Parameteres
camerainfoBag = select(bagReader,'Topic','/drivers/zed_camera/right/driver/right/camera_info') ;
cameraInfoMessages = readMessages(camerainfoBag);
cameraInfoMsg = cameraInfoMessages{1};
focalLength = cameraInfoMsg.k(1,1)
principalPoint = [cameraInfoMsg.k(3), cameraInfoMsg.k(6)]
imageSize = double ( [cameraInfoMsg.height, cameraInfoMsg.width] )
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize)

% selecting Topics and extracting the timestamps 
%ts1 = timeseries(imageBag)
%ts2 = timeseries(pcBag)
%t1 = ts1.Time 
%t2 = ts2.Time 
t1 = imageBag.MessageList.Time ;
t2 = pcBag.MessageList.Time ;

% Matching corresponding data from camera and Lidar according to their Timestamps 
k = 1;
if size(t2,1) > size(t1,1)
    for i = 1:50:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2));
        if val <= 0.1
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:50:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end

%create directories to save the valid images and point clouds
pcFilesPath = fullfile(tempdir,'PointClouds')
imageFilesPath = fullfile(tempdir,'Images')
if ~exist(imageFilesPath,'dir')
    mkdir(imageFilesPath);
end
if ~exist(pcFilesPath,'dir')
    mkdir(pcFilesPath);
end

% Extracting Images and Point clouds and save the pair under same name 
for i = 1:length(idx)
    I = rosReadImage(imageMsgs{idx(i,1)});
    pc = pointCloud(rosReadXYZ(pcMsgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    pcwrite(pc,pcFileName);
    fprintf('Matching pair %d: Image %d, Point Cloud %d\n', i, idx(i,1), idx(i,2));


end

%LidarCameraCalibrator Tool
checkerSize = 60 ; %millimeters
padding = [0 0 0 0];
lidarCameraCalibrator(imageFilesPath,pcFilesPath,checkerSize,padding)
