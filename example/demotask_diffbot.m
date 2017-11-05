function [] = demotask_diffbot()

clc
fig = figure();
cla

startup_mvtb;

% All API functions use SI units.

%% Problem parameters

% problem = [ 0, x, y           % Start point (Arena ref frame (MapRef))
%             1, x, y;          % Rescue point (unknown ref frame)
%           id1, x, y;          % Landmark 1 (unknown ref frame)
%           id2, x, y];         % Landmark 2 (unknown ref frame)
%

init_pos = [.5,.5,0];
lndmrks_need = [29, -0.2750, 2.9750;
                38, 2.2750, -3.05];
rescue_point = [-3.25,2.1];
transformed_rescue = [];

%% Initialize Simulation Class
s = VREP();

%% Using this stuff to complete a task is cheating! For testing only!
l45 = s.entity('45');
l57 = s.entity('57');
l38 = s.entity('38');
l29 = s.entity('29');
l27 = s.entity('27');
res = s.entity('RescuePoint');
mref = s.entity('MapRef');

pos45 = l45.position(mref); 
pos57 = l57.position(mref);
pos38 = l38.position(mref);
pos29 = l29.position(mref);
pos27 = l27.position(mref);
res_pos = res.position(mref);

Y = [pos45(2),pos57(2),pos38(2),pos29(2),pos27(2)];
X = [pos45(1),pos57(1),pos38(1),pos29(1),pos27(1)];



%% Continue as normal

s.startSim();
db = s.diffBot('diffBot');
ekf = demoEKF(init_pos,0.01,0.01,0.01,0.01); % (init_pose,omega_l,omega_r,omega_d,omega_b)

run = true;
state = 'initekf';
Z = getZ([4,4],init_pos);
scan = 0;
drive = 0;
step = 0;

while run
    
    odo = [0,0];  


    switch state
       
        case 'initekf'
            
            odo = [0,0];
            state = 'start_scan';
        
        case 'start_scan'
            
            if scan < 90
                
                if (90 - scan) < 15
                    step = 90 - scan;
                else
                    step = 15;
                end
                
                odo = turn(db,deg2rad(step));
                scan = scan + rad2deg(odo(2));
                
            else
                state = 'turn';
                scan = 0;
                odo = [0,0];
            end
            

        case 'turn'          
                           
            if scan < 45
                
                if (45 - scan) < 15
                    step = 45 - scan;
                else
                    step = 15;
                end
                
                odo = turn(db,-deg2rad(step));
                scan = scan + abs(rad2deg(odo(2)));
                
            else
                state = 'translate';
                scan = 0;
                odo = [0,0];
            end
            
           
        case 'translate'
                
                if drive > Z(1)
                   state = 'scan';
                else
                   if (Z(1) - drive) < .3
                       step = Z(1) - drive;
                   else
                       step = .3;
                   end
                       
                   odo = translate(db,step);
                   drive = drive + odo(1);
                end
                
        case 'scan' 
            
            if scan < 360
                odo = turn(db,deg2rad(15));
                scan = scan + 15;
            else
                state = 'find';
                odo = [0,0];
            end
            
        case 'find'
            
            % check landmarks against lndmrks_need
            
            check1 = [];
            check2 = [];
            
            if size(lndmrk_id,2) == 5 % Found all five.
                check1 = find(lndmrk_id == lndmrks_need(1,1)); 
                check2 = find(lndmrk_id == lndmrks_need(2,1)); 
            end
            
            if ~isempty(check1) && ~isempty(check2)
                
                m1t = lndmrks_need(1,2:end);
                m2t = lndmrks_need(2,2:end);
                
                a1 = 4 + (2*(check1 - 1));
                a2 = 4 + (2*(check2 - 1));
                
                m1 = mu_t(a1:a1+1);
                m2 = mu_t(a2:a2+1);
                
                trns = getTransform(m1,m2,m1t,m2t);
                transformed_rescue = transPoint(trns,rescue_point);
                
                state = 'turn2rescue';
                scan = 0;
                drive = 0;
                Z = getZ(transformed_rescue(1:2),mu_t(1:3));
                
            else
            
                state = 'done';
                disp('Not enough Landmarks found');
                disp('Rescue aborted');
                
            end
            
        case 'turn2rescue'
            
            z2abs = abs(rad2deg(Z(2)));
            if Z(2) < 0
                a = -1;
            else
                a = 1;
            end
            
            if scan < z2abs
                
                if (z2abs - scan) < 15
                    step = Z(2) - scan;
                else
                    step = 15;
                end
                
                odo = turn(db,a*deg2rad(step));
                scan = scan + abs(rad2deg(odo(2)));
            else
                state = 'move2rescue';
                scan = 0;
                odo = [0,0];
            end

        case 'move2rescue'

            if drive >= Z(1)
               state = 'done';
            else
               if (Z(1) - drive) < .3
                   step = Z(1) - drive;
               else
                   step = .3;
               end

               odo = translate(db,step);
               drive = drive + odo(1);
            end

        case 'done'
            
            disp('Run complete');
            break;
            
    end
        
    im = db.rgbdcamera.get_image();
    lnd = findLandmarks(im);
    [mu_t,sigma_t] = ekf.update(odo,lnd);
    lndmrk_id = ekf.lndmrk_id;
  
    %% Plot Actual Landmark and Robot States.
    
    halfFOV = deg2rad(90);
    coneLength = 0.2;
         
    pos_true = db.position(mref);
    ori_true = db.orientation(mref);
    x_true = pos_true;
    x_true(3) = ori_true(3) + pi;
   
    LX = x_true(1) + (coneLength)*cos(x_true(3));
    LY = x_true(2) + (coneLength)*sin(x_true(3));

    RX = x_true(1) + (coneLength)*cos(x_true(3)+halfFOV);
    RY = x_true(2) + (coneLength)*sin(x_true(3)+halfFOV);
    
    viewPortLX = mu_t(1) + (coneLength)*cos(mu_t(3));
    viewPortLY = mu_t(2) + (coneLength)*sin(mu_t(3));

    viewPortRX = mu_t(1) + (coneLength)*cos(mu_t(3)+halfFOV);
    viewPortRY = mu_t(2) + (coneLength)*sin(mu_t(3)+halfFOV);
    
    figure(fig)
    axis([-1,9,-1,9]);
    
    hold on
    
    
    scatter(X,Y,'*r');
    
    line3 = line([x_true(1),LX],[x_true(2),LY],'Color','Green');
    line4 = line([x_true(1),RX],[x_true(2),RY],'Color','Black');
    
    line1 = line([mu_t(1),viewPortLX],[mu_t(2),viewPortLY],'Color','Red');
    line2 = line([mu_t(1),viewPortRX],[mu_t(2),viewPortRY],'Color','Blue');

    

    % Plot robot cov
    plot_cov(mu_t(1:3),sigma_t(1:3,1:3),3)

    % Plot lndmrk cov
    col = ['y','m','c','g','k'];

    for i = 1:size(lndmrk_id,2)
        ln = 4+(2*(i-1));
        plot_cov(mu_t(ln:ln+1),sigma_t(ln:ln+1,ln:ln+1),3); %ln:ln+1
        scatter(mu_t(ln),mu_t(ln+1),col(i))
    end
    
    if ~isempty(transformed_rescue)
    
        scatter(transformed_rescue(1),transformed_rescue(2),'pk');
        scatter(res_pos(1),res_pos(2),'pr');
    
    end
    
    hold off
    drawnow;
    
            
         
end

s.stopSim();
s.delete();

end

function odo = getenc(db)

    enc = db.getEncoder; % Returns rotation of wheels since last encoder check.
    enc(1) = rad2deg(enc(1));
    enc(2) = rad2deg(enc(2));
    enc = enc*.000873; % Convert from degrees rotated to distance travelled.
    odo = enc;
    
end

function trans = getTransform(m1,m2,m1t,m2t)

    Ax1 = m1(1);
    Ay1 = m1(2);
    Ax2 = m2(1);
    Ay2 = m2(2);

    Bx1 = m1t(1);
    By1 = m1t(2);
    Bx2 = m2t(1);
    By2 = m2t(2);

    A = [ -By1, Bx1, 1, 0; ...
           Bx1, By1, 0, 1; ...
          -By2, Bx2, 1, 0; ...
           Bx2, By2, 0, 1; ];
    B = [Ax1; Ay1; Ax2; Ay2];

    x = linsolve(A,B);

    theta = atan2(x(1),x(2));

    trans = [theta, x(3), x(4)];

end

function p = transPoint(trans,point)

    theta = trans(1);
    AxB = trans(2);
    AyB = trans(3);

    a = [ cos(theta), -sin(theta), AxB; ...
          sin(theta),  cos(theta), AyB; ...
          0         ,  0         , 1   ];

    b = [point(1); ...
         point(2); ...
         1       ];

%     b = [point(1), point(2), 1];


%     p = dot(a,b);

      p = a*b;
  
end


function odo = turn(db,ang)

    if ang < 0
        new = [-deg2rad(50),deg2rad(50)];
    elseif ang > 0
        new = [deg2rad(50),-deg2rad(50)];
    end
    
    run = true;
    start = getenc(db);
    ang_true = 0;
    db.setMotorVel(new);

    
    while run
        
        check = getenc(db);
        current = check - start;
        c_ang = ((current(1) - current(2))/(.4));
        
        if abs(c_ang) >= abs(ang)
            db.setMotorVel([0,0]);
            run = false;
            ang_true = c_ang;
            break;
        end
        
    end
    
    pause(0.2)
    odo = [0,ang_true];

end



function odo = translate(db,dist)

if dist < 0
    new = [-deg2rad(180),-deg2rad(180)];    
elseif dist > 0
    new = [deg2rad(180),deg2rad(180)];
end


run = true;
start = getenc(db);
dist_true = 0;
db.setMotorVel(new);

    
    while run
        check = getenc(db);
        current = check - start;
        c_dist = (current(1)+current(2))/2;
        if abs(c_dist) >= abs(dist)
            db.setMotorVel([0,0]);
            run = false;
            dist_true = c_dist;
            break;
        
        end
    end  
  
    odo = [dist_true,0];
        
end


function z = getZ(goal,current)

r = sqrt((current(1) - goal(1))^2 + (current(2) - goal(2))^2);
b = wrapToPi(wrapToPi(atan2(goal(2) - current(2), goal(1) - current(1))) - current(3)); % Wrap to pi

z = [r,b];

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
           
           distance = (.10*640)/height;
           ratio = height/width;
                      
           if (distance < 5.2) && (ratio > .51)
           
               lnd(tally,1) = distance + .175;

               mid = cont(2).uc;

               fov = 45;

               if mid == 512
                   an = 0;
               else
                  an = (((fov/2)/(512/2))*(mid-(512/2)))*-1;
               end

               lnd(tally,2) = deg2rad(an);
               lnd(tally,3) = lndid;
               
           else
               
               tally = tally - 1; 
               
           end
           
        else
            i = i + 1;
        end
        
        if i > lim
            go = false;
        end
        
    
    end
    
    landmarks = lnd;

else
    
    landmarks = [];
    
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
