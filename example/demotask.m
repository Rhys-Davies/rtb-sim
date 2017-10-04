function [] = demotask()

init_pos = [.35,.35,pi/2];

s = VREP();
s.startSim();
yb = s.youbot('youBot');
%ekf = demoEKF(init_pos);


% resolution = yb.rgbdcamera.rgbsensor.resolution %This is a bit rediculous. Minor re-arrange may be beneficial.
% per_ang = yb.rgbdcamera.rgbsensor.fov
% 
% 
%  ratio=resolution(1)/resolution(2);
%  if (ratio>1)
%      fov=2*atan(tan(per_ang/2)/ratio);
%  else
%      fov=per_ang;
%  end
%  
%  fov = rad2deg(fov)

%
%

%



run = true;

while run

%     motion controller
%     tic
%     
%     while toc < 0.5
%     end
%     
%     % stop
    
    im = yb.rgbdcamera.image();
    lnd = findLandmarks(im);
    %pos = ekf.update


    pause (3);
end


% Initial pose wrt robot ref frame is [.35,.35,pi/2]
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
    contourXsort = contour_list(:,index);
    colourXsort = colour_list(index,:);
    
    list = [transpose(contourXsort.uc) , transpose(contourXsort.vc)];

    
    lim = size(contourXsort,2);
    lnd = [];
    tally = 0;
    i = 1;
    go = true;
    
    while go
    
        height = contourXsort(i).vmax - contourXsort(i).vmin;
        width = contourXsort(i).umax - contourXsort(i).umin;
        upperY = contourXsort(i).vc + (height*2);
        lowerY = contourXsort(i).vc - (height*2);
        upperX = contourXsort(i).uc + width/2;
        lowerX = contourXsort(i).uc - width/2;
    
        filt = list(:,1)>lowerX & list(:,1)<upperX & list(:,2)>lowerY & list(:,2)<upperY;
        
        if sum(filt) == 3
           i = find(filt,1,'last') + 1;
           tally = tally + 1;
           id = colourXsort(filt==1);
           cont = contourXsort(:,filt==1);
           [~,ind] = sortrows(transpose(cont.vc),1,'descend');
           cont = cont(:,ind);
           id = id(ind);
           lndid = bi2de([de2bi(id(1),2),de2bi(id(2),2),de2bi(id(3),2)]);
           lnd(tally,1) = lndid;
           lnd(tally,2) = (.10*640)/height;
           
           mid = cont(2).uc;
           
           fov = 45;

           if mid == 512
               an = 0;
           else
              an = ((fov/2)/(512/2))*(mid-(512/2));
           end
                
           lnd(tally,3) = an;
           
        else
            i = i + 1;
        end
        
        if i > lim
            go = false;
        end
        
    
    end
    
    
    
    
    landmarks = lnd;

end



end



function [thresh_red, thresh_green, thresh_blue] = thresholdImage(image, ie, id)

%% Threshhold

impro = rgb2hsv(image); % Convert from RGB to HSV Colorspace

figure(5)
imshow(impro)

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
