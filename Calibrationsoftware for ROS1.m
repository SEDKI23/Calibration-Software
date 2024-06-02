%Retrieve information from the bag file ROS1
path = "/home/bensalem/Documents/new_data/rosbag2_2024_01_15-14_47_34.bag" ;
bag = rosbag(path); 


% Select image and point cloud messages from the rosbag and select a subset of messages from the file by using the appropriate topic names.
imageBag = select(bag,'Topic','/drivers/zed_camera/right/driver/right/image_rect_color') ;
pcBag = select(bag,'Topic','/drivers/ouster_lidar/points') ;

% Read all the messages.
imageMsgs = readMessages(imageBag) ;
pcMsgs = readMessages(pcBag) ;

% Intrinsc Parameteres
camerainfoBag = select(bag,'Topic','/drivers/zed_camera/right/driver/right/camera_info') ;
cameraInfoMessages = readMessages(camerainfoBag);
cameraInfoMsg = cameraInfoMessages{1};
focalLength = cameraInfoMsg.K(1,1)
principalPoint = [cameraInfoMsg.K(3), cameraInfoMsg.K(6)]
imageSize = double ( [cameraInfoMsg.Height, cameraInfoMsg.Width] )
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize)


% selecting Topics and extracting the timestamps 
ts1 = timeseries(imageBag);
ts2 = timeseries(pcBag);
t1 = ts1.Time
t2 = ts2.Time


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
    I = readImage(imageMsgs{idx(i,1)});
    pc = pointCloud(readXYZ(pcMsgs{idx(i,2)}));
    n_strPadded = sprintf('%04d',i) ;
    pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
    imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.png');
    imwrite(I,imageFileName);
    pcwrite(pc,pcFileName);
end


% Load images using imageDatastore.
imds = imageDatastore('/tmp/Images');
imageFileNames = imds.Files;

% Load point cloud files.
pcds = fileDatastore('/tmp/PointClouds','ReadFcn',@pcread);
ptCloudFileNames = pcds.Files;


% Square size of the checkerboard in mm.
squareSize = 60;                                     

% Set random seed to generate reproducible results.
rng('default')

% Extract checkerboard corners from the images.
[imageCorners3d,checkerboardDimension,dataUsed] = ...
    estimateCheckerboardCorners3d(imageFileNames,intrinsics,squareSize);

% Remove the unused image files.
imageFileNames = imageFileNames(dataUsed);           

% Filter the point cloud files that are not used for detection.
ptCloudFileNames = ptCloudFileNames(dataUsed);

% Extract ROI from the detected checkerboard image corners.
roi = helperComputeROI(imageCorners3d,5);

% Extract checkerboard plane from point cloud data.
[lidarCheckerboardPlanes,framesUsed,indices] = detectRectangularPlanePoints( ...
    ptCloudFileNames,checkerboardDimension,RemoveGround=true,ROI=roi);
imageCorners3d = imageCorners3d(:,:,framesUsed);

% Remove ptCloud files that are not used.
ptCloudFileNames = ptCloudFileNames(framesUsed);

% Remove image files that are not used.
imageFileNames = imageFileNames(framesUsed);

%estimate the rigid transformation matrix between the lidar sensor and the camera
[tform,errors] = estimateLidarCameraTransform(lidarCheckerboardPlanes, ...
    imageCorners3d,intrinsics);



% Display checkerboard corners.
helperShowImageCorners(imageCorners3d,imageFileNames,intrinsics)
%Visualize the detected checkerboard
helperShowCheckerboardPlanes(ptCloudFileNames,indices)
%isualize the lidar and the image data fused together
helperFuseLidarCamera(imageFileNames,ptCloudFileNames,indices, ...
   intrinsics,tform)
helperShowError(errors)

eulZYX = rotm2eul(tform.R) ;
xyz=tform.Translation ;
save ("RESULT.mat",'eulZYX','xyz') ;
load("RESULT.mat") ;


















