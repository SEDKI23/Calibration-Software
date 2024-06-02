%% Select which Camera/Lidar to Calibrate; Read all Rosbags for every Camera and Position

LIDAR = input("WHICH LIDAR IS USED ? Please select : OUSTER , AEVA OR VELARRAY: ","s");
CAMERA = input("Which Camera is used : ","s");

deleteExistingFiles('/home/bensalem/Documents/ROSBAGS/EXPORT/PointClouds', '.pcd'); % Function to delete PCD files
deleteExistingFiles('/home/bensalem/Documents/ROSBAGS/EXPORT/Images', '.png'); % Function to delete PNG files

%Camera Topic 
CAMERA_TOPIC = "/drivers/zed_camera/"+CAMERA+"/driver/left/image_rect_color/compressed" ;
CAMERA_INFO_TOPIC = "/drivers/zed_camera/"+CAMERA+"/driver/left/camera_info" ;

%Saving Lidar Topic and the Range of the lidars to the corresponding Camera
if LIDAR=="AEVA" 
    LIDAR_TOPIC ='/drivers/aeva_lidar/point_cloud' ;

    if CAMERA~="front_center" && CAMERA~="front_left" && CAMERA~="front_right"
        disp('Wrong Lidar-Camera Combination, Out of Range')
        return ;
    end

elseif LIDAR=="OUSTER"
    LIDAR_TOPIC='/drivers/ouster_lidar/points' ;

elseif LIDAR=="VELARRAY"
    LIDAR_TOPIC='/drivers/velarray_lidar/point_cloud';

        if (CAMERA~="rear_center") && (CAMERA~="rear_right") && (CAMERA~="rear_left")
            disp('Wrong Lidar-Camera Combination, Out of Range')
            return ;                
        end

else
    return ;
end

%Camera differentiation and SAVING The Path of ROSBAGS
switch CAMERA
    case 'front_center'
        
        path ="/home/bensalem/Documents/ROSBAGS/front_center" ;

    case 'front_right'

        path ="/home/bensalem/Documents/ROSBAGS/front_right" ;

     
    
    case 'front_left'

        path = "/home/bensalem/Documents/ROSBAGS/front_left" ;


    
    case 'mid_right'

        path = "/home/bensalem/Documents/ROSBAGS/mid_right";
        
   
    case 'mid_left'
        
        path = "/home/bensalem/Documents/ROSBAGS/mid_left";
   
       

    case 'rear_center'
        
        path = "/home/bensalem/Documents/ROSBAGS/rear_center";        
   
        
    case 'rear_left'
                
        path = "/home/bensalem/Documents/ROSBAGS/rear_left";
    
    
      
    case 'rear_right'

        path = "/home/bensalem/Documents/ROSBAGS/rear_right";
             
    
end

 %% Read all the messages and Match data to get Image/PCD Pairs

% Define suffixes for bag files
bag_suffixes = { "_ll","_l", "_c", "_r", "_rr"}; 
%mid_right_l is missing so remove the "_l",

% Loop through each ROSBAG
for i = 1:length(bag_suffixes)
    suffix = bag_suffixes{i};
    
    bagReader = ros2bagreader(path+'/'+CAMERA+suffix); 
    imageBag= select(bagReader,'Topic',CAMERA_TOPIC);
    pcBag = select(bagReader,'Topic',LIDAR_TOPIC) ;

    imageMsgs = readMessages(imageBag) ;
    pcMsgs = readMessages(pcBag) ;


    % Selecting Topics and extracting the timestamps 
    t1 = imageBag.MessageList.Time ;
    t2 = pcBag.MessageList.Time ;
    
    % Find the first matching pair based on timestamps
    [val, indx] = min(abs(t1(1) - t2));
    
    % Extract just the first matching image and point cloud
    I = rosReadImage(imageMsgs{1});
    pc = pointCloud(rosReadXYZ(pcMsgs{indx}));
    
    
    % Save the pair under the same name
    n_strPadded = sprintf('%04d', i);  % Format with leading zeros (optional)
    pcFileName = strcat('/home/bensalem/Documents/ROSBAGS/EXPORT/PointClouds', '/', n_strPadded, '.pcd');
    imageFileName = strcat('/home/bensalem/Documents/ROSBAGS/EXPORT/Images', '/', n_strPadded, '.png');
    imwrite(I, imageFileName);
    pcwrite(pc, pcFileName);
    fprintf('Matching pair: Image 1, Point Cloud %d\n', indx);

    camerainfoBag = select(bagReader,'Topic',CAMERA_INFO_TOPIC) ;


end
 
 %% Read and save intrinsics Parameteres from the appropriate Topic camera_info

 cameraInfoMessages = readMessages(camerainfoBag);
 cameraInfoMsg = cameraInfoMessages{1};
 focalLength = cameraInfoMsg.k(1,1)
 principalPoint = [cameraInfoMsg.k(3), cameraInfoMsg.k(6)]
 imageSize = double ([cameraInfoMsg.height, cameraInfoMsg.width] )
 intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize)


%%  Extract the corners of the Checkerboard

% Load images using imageDatastore.
imds = imageDatastore('/home/bensalem/Documents/ROSBAGS/EXPORT/Images');
imageFileNames = imds.Files;

% Load point cloud files.
pcds = fileDatastore('/home/bensalem/Documents/ROSBAGS/EXPORT/PointClouds','ReadFcn',@pcread);
ptCloudFileNames = pcds.Files;


% Square size of the checkerboard in mm.
squareSize = 60;                                     

% Set random seed to generate reproducible results.
rng('default')

% Extract checkerboard corners from the images.
[imageCorners3d,checkerboardDimension,dataUsed] = ...
    estimateCheckerboardCorners3d(imageFileNames, intrinsics, squareSize, "Padding", [40 30 40 30] ,'MinCornerMetric',0.2)

% Remove the unused image files.
imageFileNames = imageFileNames(dataUsed);           

% Filter the point cloud files that are not used for detection.
ptCloudFileNames = ptCloudFileNames(dataUsed);



%% Extract ROI from the detected checkerboard image corners for 3 diffrent Lidars

switch LIDAR 

    case 'OUSTER'
    %Region of Interest :
        if  CAMERA == "front_center" ||  CAMERA=="front_right" ||  CAMERA=="front_left"
        roi = helperComputeROIOuster_vorne(imageCorners3d,2); 
        elseif CAMERA == "rear_center" || CAMERA == "rear_left"
        roi = helperComputeROIOuster_hinten(imageCorners3d,2) 
        elseif CAMERA == "mid_left"
        roi = [[-4, 4, -5, 0, -3, 6]] 
        elseif CAMERA == "mid_right"
        roi = [[-2, 2, 2, 6, -3, 6]]   
        elseif CAMERA == "rear_right"
        roi = [[0, 6, 0, 4, -3, 6]]    
        end

    %Parameters : 
    minDistance = 0.1;
    DimensionTolerance=0.2;
    
    

    case 'AEVA'
    %Region Of Interest
    if CAMERA=="front_right"
        roi = [[2, 6, -4, 0, -3, 1]] ;
    else
        roi = helperComputeROI(imageCorners3d, 2);
    end


    %Parameters : 
    minDistance = 0.05;
    DimensionTolerance=0.2;
    
    case 'VELARRAY'
    %Region of Interest : 
    roi = helperComputeROI(imageCorners3d, 5);

    %Parameters : 
    minDistance = 0.1;
    DimensionTolerance=0.2;


end
%% Plot The Pointclouds in the Region of Interest 

for j=1:sum(dataUsed(:) == 1)
    ptCloud = pcread(ptCloudFileNames{j});
    indices = findPointsInROI(ptCloud,roi);
    ptCloudB = select(ptCloud,indices);
    %figure(i)
    hold on
    pcshow(ptCloudB);
    hold off
end
title ('The Pointclouds in the Region of Interest of every Checkerboard ');

%% Extracting The Checkerboard Planers and Estimating transformation matrix between the lidar and the camera

% Extract checkerboard plane from point cloud data.
[lidarCheckerboardPlanes,framesUsed,indices] = detectRectangularPlanePoints( ...
    ptCloudFileNames, checkerboardDimension, RemoveGround=false, ROI=roi, minDistance=minDistance, DimensionTolerance=DimensionTolerance, Verbose=1)
imageCorners3d = imageCorners3d(:,:,framesUsed) ;

% Remove ptCloud files that are not used.
ptCloudFileNames = ptCloudFileNames(framesUsed);

% Remove image files that are not used.
imageFileNames = imageFileNames(framesUsed);

%estimate the rigid transformation matrix between the lidar sensor and the camera
[tform,errors] = estimateLidarCameraTransform(lidarCheckerboardPlanes, imageCorners3d, intrinsics);

 %% Plot whole pointcloud on image

%loop over all pairs and plot in one matlab figure
for j = 1:sum(framesUsed(:) == 1)
    ptCloud = pcread(ptCloudFileNames{j});
    %roi = [[-10, 10, -20, 20, -1, 4]];
    %Cloud = select(ptCloud,findPointsInROI(ptCloud,roi));
    img = imread(imageFileNames{j});
    imPts = projectLidarPointsOnImage(ptCloud,intrinsics,tform);
    figure(i)
    imshow(img)
    hold on
    plot(imPts(:,1),imPts(:,2),'.','Color','r')
    hold off
    pause(2)
end
%% Visualize the Result of the Calibration :

%Display checkerboard corners.
helperShowImageCorners(imageCorners3d,imageFileNames,intrinsics)

%Visualize the detected checkerboard
helperShowCheckerboardPlanes(ptCloudFileNames,indices)

%visualize the lidar and the image data fused together
helperFuseLidarCamera(imageFileNames,ptCloudFileNames,indices,intrinsics,tform)

helperShowError(errors)





%% TF
if LIDAR =="OUSTER"

%  Given by camera manufacturer
A = [0.000 1.000 0.000 0.060;
     -0.000 0.000 1.000 -0.015;
     1.000 0.000 0.000 0.010;
     0.000 0.000 0.000 1.000];

% stores information about 3-D rigid geometric transformation 
tform_left_optical_to_camera_base_link = rigidtform3d(A);

% Invert export Eulerangles and Translation Vector and print the tf_base link to left_optical_frame 
% (from the manufacturer so it is the same for all 8 cameras).
eulZYX = rotm2eul(tform_left_optical_to_camera_base_link.invert.R);
xyz = tform_left_optical_to_camera_base_link.invert.Translation;
fprintf('The %s_LIDAR_%s_CAMERA xyz Yaw,Pitch,Roll is : \n%s\n', LIDAR, CAMERA, mat2str([xyz, eulZYX]));


% Invert Matrix (Ouster is the Reference so the Transofrmation is from The
% Ouster Frame id to the the child optical frame 
 tform_inv = invert(tform);

% Multiply by the result of the calibration
tform_to_camera_base_link = rigidtform3d(tform_inv.A * tform_left_optical_to_camera_base_link.A);


% Euler Transformation and print the
% calibrated_tf_ouster to _base_link Transform 
eulZYX = rotm2eul(tform_to_camera_base_link.R) ;
xyz = tform_to_camera_base_link.Translation ;
fprintf('The %s_LIDAR_%s_CAMERA xyz Yaw,Pitch,Roll is : \n%s\n', LIDAR, CAMERA, mat2str([xyz, eulZYX]));



else
eulZYX = rotm2eul(tform.R) ;
xyz=tform.Translation ;
fprintf('The %s_LIDAR_%s_CAMERA yxz Yaw,Pitch,Roll is : \n%s\n', LIDAR, CAMERA, mat2str([xyz,eulZYX]));

end


%% Detele Existring Files 
function deleteExistingFiles(folderPath, extension)
  % This function deletes all files with the specified extension in the given folder.
  files = dir(fullfile(folderPath, ['*', extension]));
  for i = 1:length(files)
    delete(fullfile(folderPath, files(i).name));
  end
end





















