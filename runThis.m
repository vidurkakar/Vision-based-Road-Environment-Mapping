clc;
clear all;
close all;

% Load the dash-cam footage and cifar10NetRCNN
workingDir = 'laneFrame';
mkdir(workingDir);
mkdir(workingDir, 'images');
v = VideoReader('project_video.mp4');
%v = VideoReader('video_toronto.mp4');
%v = VideoReader('video_3.mp4');
%data = load('fasterRCNNVehicleTrainingData.mat');
%detector = data.detector;
load('cifar10NetRCNN.mat');

horizon = 470;
%horizon = 550;

% For each frame apply RCNN to detect vehicle bounding boxes in the frame
% followed by lane detection

for i = 1:v.Duration * v.FrameRate
    frame = read(v, i);
    %[bboxes, scores] = detect(detector, frame);
	
	% Vehicle Detection
    [bboxes, scores] = detect(cifar10NetRCNN, frame);
	
	%Lane Detection
    [laneFrame, goodFrame] = getLane(frame, horizon);
    if goodFrame == 0
        continue;
    end
	
	% Insert the detected lane on the original frame
    laneFrame = imresize(laneFrame, [720 - horizon, 1280]);
    frameSubset = frame(horizon+1:720, :, :);
    mask = ~logical(laneFrame);
    frameSubset = im2double(frameSubset) .* mask + laneFrame;
    %imshow(frameSubset);
    frame = im2double(frame);
    frame(horizon+1:720, :, :) = frameSubset;
    
	% Thresholding of detected bounding boxes to reduce false positives
	
    idx = scores > 0.875;
    bboxes = bboxes(idx, :);
    scores = scores(idx);
	
	% Inserting bounding boxes on vehicles
	
    if(bboxes)
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes, '', 'LineWidth', 5); % scores);
    end
	
	% Save each frame in directory
	
    filename = [sprintf('%04d', i) '.jpg'];
    fullname = fullfile(workingDir, 'images', filename);
    imwrite(frame, fullname);
end

imageNames = dir(fullfile(workingDir,'images','*.jpg'));
imageNames = {imageNames.name}';

% Transfomr the final frames into the video
outputVideo = VideoWriter(fullfile(workingDir,'out.avi'));
outputVideo.FrameRate = v.FrameRate;
open(outputVideo)

for ii = 1:length(imageNames)
   img = imread(fullfile(workingDir,'images',imageNames{ii}));
   writeVideo(outputVideo,img)
end

close(outputVideo)

%imshow(getLane(read(v, 10)));