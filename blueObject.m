%% Setup ROS Communication
rosIP = "192.168.2.130";  % Your ROS master IP

rosshutdown;
rosinit(rosIP, 11311);
pause(2);

% Setup subscribers
rgbSub = rossubscriber('/camera/rgb/image_raw', 'sensor_msgs/Image', 'DataFormat', 'struct');
pcSub  = rossubscriber('/camera/depth/points', 'sensor_msgs/PointCloud2', 'DataFormat', 'struct');

pause(2);

%% Capture RGB image and PointCloud
rgbMsg = receive(rgbSub, 5);
pcMsg  = receive(pcSub, 5);

rgbImg = rosReadImage(rgbMsg);
ptCloud = rosReadXYZ(pcMsg);

figure(1);
imshow(rgbImg);
title('Captured RGB Image');

%% Object Detection 
hsvImg = rgb2hsv(rgbImg);


blueMask = hsvImg(:,:,1) > 0.56 & hsvImg(:,:,1) < 0.70;
bsatMask = hsvImg(:,:,2) > 0.35;
valm = hsvImg(:,:,3) > 0.3;
mask = blueMask & bsatMask & valm;

mask = imclose(mask, strel('disk', 5));
mask = imfill(mask, 'holes');
mask = bwareaopen(mask, 300);

stats = regionprops(mask, 'BoundingBox', 'Centroid', 'Area');
if isempty(stats)
    error(' No object detected!');
end

[~, idx] = max([stats.Area]);
bbox = stats(idx).BoundingBox;
centroid = stats(idx).Centroid;

figure(2);
imshow(rgbImg);
hold on;
rectangle('Position', bbox, 'EdgeColor', 'r', 'LineWidth', 2);
plot(centroid(1), centroid(2), 'g*');
title('Detected Object');
hold off;

disp(' Object detected.');

%% Extract 3D Coordinates
u = round(centroid(1));
v = round(centroid(2));

u = min(max(u, 1), size(ptCloud, 2));
v = min(max(v, 1), size(ptCloud, 1));

xyz = squeeze(ptCloud(v, u, :));

if any(isnan(xyz)) || norm(xyz) < 1e-3
    error(' Invalid or missing 3D point.');
end

b = xyz(1);

disp(' 3D position extracted.');

%% Object Classification (fixed)
bboxMask = false(size(mask));
rowRange = round(bbox(2)) : min(round(bbox(2)+bbox(4)), size(mask,1));
colRange = round(bbox(1)) : min(round(bbox(1)+bbox(3)), size(mask,2));
bboxMask(rowRange, colRange) = true;

maskedHue = hsvImg(:,:,1) .* double(bboxMask);
validHue = maskedHue(maskedHue > 0);
meanHue = mean(validHue);



% Classification logic
if meanHue > 0.25 && meanHue < 0.4
    objectClass = 'Can';  
elseif (meanHue > 0.55 && meanHue < 0.7)
    objectClass = 'Bottle'; 
else
    objectClass = 'Unknown';
end

%% Final Output
fprintf('\nThe object is classified as: %s\n', objectClass);
fprintf(' Distance from the object to the cameera: D=%.3f m\n', b);
