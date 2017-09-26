function [] = demotask()
% Initial pose wrt robot ref frame is [.35,.35,pi/2]
end




function [landmarks] = findLandmarks(foundRed,foundGreen,foundBlue)



end


function [foundRed, foundGreen, foundBlue] = findBlobs(thresh_red, thresh_green, thresh_blue)

foundRed = iblobs(thresh_red,'area',[350,5000],'boundary', 'class',1);
foundGreen = iblobs(thresh_green, 'area', [350,5000], 'boundary', 'class',1);
foundBlue = iblobs(thresh_blue, 'area', [350,5000], 'boundary', 'class',1);


end


function [thresh_red, thresh_green, thresh_blue] = thresholdImage(image, ie, id)

%% Threshhold

impro = rgb2hsv(image); % Convert from RGB to HSV Colorspace

%% Set Blue Threshhold
thresh_blue_H = impro(:,:,1) > 0.43 & impro(:,:,1) < 0.6; %Threshold the HUE component of the image
thresh_blue_S = impro(:,:,2) > 0.15 & impro(:,:,2) < 0.9; %Threshold the SATURATION component of the image.
thresh_blue_V = impro(:,:,3) > 0.11 & impro(:,:,3) < 0.9; %Threshold the VALUE component of the image.

thresh_blue = thresh_blue_H.*thresh_blue_S.*thresh_blue_V; %Element wise multiply the bitmask for all three.

imageBlue = imerode(thresh_blue, ones(ie,ie));
imageBlue = imdilate(imageBlue,ones(id,id));

%% Set Red Threshhold

thresh_red_H = (impro(:,:,1) > 0.92 & impro(:,:,1) < 1) | (impro(:,:,1) >= 0 & impro(:,:,1) < 0.04);
thresh_red_S = impro(:,:,2) > 0.4 & impro(:,:,2) < 0.96;
thresh_red_V = impro(:,:,3) > 0.33 & impro(:,:,3) < 0.76;

thresh_red = thresh_red_H.*thresh_red_S.*thresh_red_V;

imageRed = imerode(thresh_red, ones(ie,ie));
imageRed = imdilate(imageRed,ones(id,id));

%% Set Green Threshhold

thresh_green_H = impro(:,:,1) > 0.31 & impro(:,:,1) < 0.43;
thresh_green_S = impro(:,:,2) > 0.41 & impro(:,:,2) < 1;
thresh_green_V = impro(:,:,3) > 0.1 & impro(:,:,3) < 0.8;

thresh_green = thresh_green_H.*thresh_green_S.*thresh_green_V;

imageGreen = imerode(thresh_green,ones(ie,ie));
imageGreen = imdilate(imageGreen,ones(id,id));

thresh_blue = imageBlue;
thresh_green = imageGreen;
thresh_red = imageRed;

end
