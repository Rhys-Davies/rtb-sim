function sol

%% Load the Problem
f = load('instructions.mat');
inst = f.inst;

shapes = inst.shapes;
color = inst.color;
colorname =  inst.colorname;
picfn = inst.picture;

%% Initialize the simulator and robot

sim = VREP(); % Default is to attempt connection to localhost
yb = sim.youbot('youBot'); % TRS youBot is a modified version of the stock VREP one.
sim.startSim();

timestep = 0.05;

%% Initialize the display window

subplot(211);
drawnow;

%% Explore

% Roam
% EKF Slam (Does RTB EKF handle data association?)
% D* Path planning
% 
%

%
% Goal is to locate n circular objects: n = number of pictures (# of
% baskets) + 2 (tables). 
% Start facing the tables. Map both immediately. Robot initial origin =
% (0,0) of the map


%%
% notdone = true;

% while notdone == true
%     % Main loop
%     
%
%     Take sensor reading, do EKF SLAM
%     [pts, contacts] = yb.hokuyo.scan; % contacts are the laser returns

%      if plotData
%         
%         % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo.
%         in = inpolygon(X, Y, [s1(1), pts(1, :), s2(1)], [s1(2), pts(2, :), s2(2)]);
%                     
%         % Plot those points. Red stars: the obstacles. Blue lines: the
%         % visibility range from the Hokuyo sensor. 
%         % The youBot is indicated with two dots: the green one corresponds to the rear, the blue one to the Hokuyo
%         % sensor position. 
%         subplot(211)
%          plot(pts(1, contacts), pts(2, contacts), '*r',...
%              [s1(1), pts(1, :), s2(1)], [s1(2), pts(2, :), s2(2)], 'b',...
%              0, 0, 'og', s1(1), s1(2), 'ob', s2(1), s2(2), 'ob');
%         axis([-5.5, 5.5, -5.5, 2.5]);
%         axis equal;
%         drawnow;
%                       
%     end








%     
%     state = 'explore';
%     
%     
%     switch (state)
%         
%         case 'explore'
%             
%           while true
%           end
%             
%         case 'transport'
%
%           tstate = 'retrieve';
%
%           while true
%           
%               switch (tstate)
%                   case tstate == 'retrieve'
%
%                       % Travel to table
%                       % Position self at table
%                       % If no items, break, notdone == false 
%                       % Grab item
%                       
%
%                       tstate = 'deposit';
%
%
%                   case tstate == 'deposit'
%
%                       % Travel to room (or basket, milestone dependant)
%                       % Drop                       
%
%                       tstate = 'retrieve';
%               end
%
%           end
%
%     end
%        
% end

end

