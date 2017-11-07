function viscal()

clc

startup_mvtb;

%% Initialize Simulation Class
s = VREP();

%% Using this stuff to complete a task is cheating! For testing only!
l45 = s.entity('45');
l57 = s.entity('57');
l38 = s.entity('38');
l29 = s.entity('29');
l27 = s.entity('27');
mref = s.entity('MapRef');
rpos = s.entity('RescuePoint');
rres = s.entity('RescueRef');



%% Continue as normal

s.startSim();
db = s.diffBot('diffBot');

%     
im = db.rgbdcamera.get_image();
imshow(im);
lnd = findLandmarks(im)

end

function [landmarks] = findLandmarks(image)
% landmarks = [range,bearing,id]

ie = 1;
id = 2;


%% find r, g and b blobs

[thresh_red, thresh_green, thresh_blue] = thresholdImage(image,ie,id);

foundRed = iblobs(thresh_red,'area',[50,50000],'boundary', 'class',1);
foundGreen = iblobs(thresh_green, 'area', [50,50000], 'boundary', 'class',1);
foundBlue = iblobs(thresh_blue, 'area', [50,50000], 'boundary', 'class',1);

% figure(1)
% idisp(image);
% figure(2)
% idisp(thresh_red);
% figure(3)
% idisp(thresh_green);
% figure(4)
% idisp(thresh_blue);


%% sort by 

% r = 01 = 1
% g = 10 = 2
% b = 11 = 3

totalFound = size(foundRed,2) + size(foundBlue,2) + size(foundGreen,2);

if totalFound > 2 % Needs to be at least 3 for one landmark


    contour_list = [foundRed,foundGreen,foundBlue];
    colour_list = [ones(size(foundRed,2),1);ones(size(foundGreen,2),1)*2;ones(size(foundBlue,2),1)*3];
    
    t = transpose(contour_list.uc);
    [~,index] = sortrows(t,1,'ascend'); 
    contourXsort = contour_list(:,index)
    colourXsort = colour_list(index,:);
    
    list = [transpose(contourXsort.uc) , transpose(contourXsort.vc)];

    
    height = [];
    i = 1;
    go = true;
    
    while go
        top = contourXsort(i).vmax
        bottom = contourXsort(i).vmin
        height(i,1) = top - bottom
        i = i + 1;
        
        if i > 3
            go = false;
        end
        
    end

    landmarks = height;

end

end



function [thresh_red, thresh_green, thresh_blue] = thresholdImage(image, ie, id)

%% Threshhold

impro = rgb2hsv(image); % Convert from RGB to HSV Colorspace

%% Set Blue Threshhold
thresh_blue_H = impro(:,:,1) > 0.6 & impro(:,:,1) < 0.7; %Threshold the HUE component of the image
thresh_blue_S = impro(:,:,2) > 0.15 & impro(:,:,2) < 1.01; %Threshold the SATURATION component of the image.
thresh_blue_V = impro(:,:,3) > 0.11 & impro(:,:,3) < 1.01; %Threshold the VALUE component of the image.

thresh_blue = thresh_blue_H.*thresh_blue_S.*thresh_blue_V; %Element wise multiply the bitmask for all three.

imageBlue = imerode(thresh_blue, ones(ie,ie));
imageBlue = imdilate(imageBlue,ones(id,id));

%% Set Red Threshhold

thresh_red_H = (impro(:,:,1) > 0.92 & impro(:,:,1) < 1) | (impro(:,:,1) >= 0 & impro(:,:,1) < 0.1);
thresh_red_S = impro(:,:,2) > 0.4 & impro(:,:,2) < 1.01;
thresh_red_V = impro(:,:,3) > 0.33 & impro(:,:,3) < 1.01;

thresh_red = thresh_red_H.*thresh_red_S.*thresh_red_V;

imageRed = imerode(thresh_red, ones(ie,ie));
imageRed = imdilate(imageRed,ones(id,id));

%% Set Green Threshhold

thresh_green_H = impro(:,:,1) > 0.31 & impro(:,:,1) < 0.43;
thresh_green_S = impro(:,:,2) > 0.41 & impro(:,:,2) < 1.01;
thresh_green_V = impro(:,:,3) > 0.1 & impro(:,:,3) < 1.01;

thresh_green = thresh_green_H.*thresh_green_S.*thresh_green_V;

imageGreen = imerode(thresh_green,ones(ie,ie));
imageGreen = imdilate(imageGreen,ones(id,id));

thresh_blue = imageBlue;
thresh_green = imageGreen;
thresh_red = imageRed;

end
