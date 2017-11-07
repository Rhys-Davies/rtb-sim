clear all
close all
clc

%% TRS Task API Functionality test
% Rewrite of youbot.m from the TRS repository to use rtb-sim in order to
% verify the interface has reached feature parity with the existing trs
% task codebase.
% Original code found here: https://github.com/ULgRobotics/trs
% Original code (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017, Mathieu Baijot 2017.
% Modified Rhys Davies 2017
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

sim = VREP(); % Default is to attempt connection to localhost
yb = sim.youBotTRS('youBot'); % TRS youBot is a modified version of the stock VREP one.
mo_ctrl = yb_motion_controller(20,12,10,4,0.05);
sim.startSim();

% Starting pose of the arm.
startPose =  [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0]; 
% Preset pickup pose of the arm.
pickupPose = [90 * pi / 180, 19.6 * pi / 180, 113 * pi / 180, - 41 * pi / 180, 0];


% Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
% They are adapted at each iteration by the code. 
forwBackVel = 0;
leftRightVel = 0;
rotVel = 0;
prevOri = 0; 
prevLoc = 0;

pause(2); %Give everything time to settle


yb.arm.set_state(startPose);

% Initialise the plot. 
plotData = true;
if plotData
    subplot(211);
    drawnow;

    % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
    % see, by selecting the points within this mesh that are within the visibility range. 
    [X, Y] = meshgrid(-5:.25:5, -5.5:.25:2.5);
    X = reshape(X, 1, []);
    Y = reshape(Y, 1, []);
end

gripperHome = yb.gripper.position(yb.arm_ref);

% Initialise state machine
fsm = 'rotate';

%% Start the Demo
while true
    
    tic
    
    youbotPos = yb.position;
    youbotOrient = yb.ref.orientation;
    
    [pts, contacts] = yb.hokuyo_scan;% contacts are the obstacles ... .scan('plot',figure1)
    
    %% Remove stuff like this: Plotting handled by hokuyo class
    if plotData
        % Read data from the Hokuyo
        
        s = yb.plot_ref;
        
        subplot(211)
         
         plot(pts(1, contacts), pts(2, contacts), '*r',...
              [s(1), pts(1, :), s(1)], [s(2), pts(2, :), s(2)], 'b',...
              0, 0, 'og', s(1), s(2), 'ob', s(1), s(2), 'or');        
         
        axis([-5.5, 5.5, -5.5, 2.5]);
        axis equal;
        drawnow;
                      
    end
    
    angl = -pi/2;

%% Apply the state machine.

    switch (fsm)
        case 'rotate' 
            %% First, rotate the robot to go to one table. 
            
            rotVel = angdiff(angl, youbotOrient(3))/2;
            
            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotOrient(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotOrient(3))) < .01 / 180 * pi)
                rotVel = 0;
                fsm = 'drive';
            end

            prevOri = youbotOrient(3);   
        case 'drive'
            %% Then, make it move straight ahead until it reaches the table. 
            % The further the robot, the faster it drives. (Only check for the first dimension.)

            forwBackVel = -(youbotPos(1) + 3.167)/2;
            % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
            % a specific location before moving on to the next state.
            if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevLoc) < .001) && (abs(forwBackVel) < 0.05) 
                forwBackVel = 0;
                % Change the orientation of the camera
                yb.rgbdcamera.set_orientation([0 0 pi/4],yb.ref.id);
                % Move the arm to the preset pose.
                yb.arm.set_state(pickupPose);

                fsm = 'snapshot';
            end
            prevLoc = youbotPos(1);
        case 'snapshot'
            %% Read Data from Range Camera
            pts = yb.get_point_cloud;
            
            pts = pts(1:3, pts(4, :) < 1); % [x,y,z], pts(4, :) < 1); 4th row elements < 1

            if plotData
                subplot(223)
                %plot3(pts(1, :), pts(3, :), pts(2, :), '*'); % Plotting Z as Y and Y as Z.
                plot3(pts(1, :), pts(2, :), pts(3, :), '*');
                axis equal;
                view([-169 -46]);
            end
            
            image = yb.get_image;
            
            if plotData
                subplot(224)
                imshow(image);
                drawnow;
            end

            % Next state. 
            fsm = 'extend';
            
        case 'extend'        
            %% Move the arm to face the object.
            % Get the arm position. 
            
            tpos = yb.gripper.position(yb.arm_ref.id); % 
            
            % If the arm has reached the wanted position, move on to the next state. 
            
            if norm(tpos - [0.3259 -0.0010 0.2951]) < .002
                
                % Set the inverse kinematics (IK) mode to position AND orientation. 
                yb.setkinematicmode(2);
                
                fsm = 'reachout';
            end
        case 'reachout'
            %% Move the gripper tip along a line so that it faces the object with the right angle.
            % Get the arm tip position. (It is driven only by this position, except if IK is disabled.)
            
            
            tpos = yb.gripper.position(yb.arm_ref.id); % (referenced to 'youBot_ref') 'youBot_gripperPositionTip'
            
            % If the tip is at the right position, go on to the next state. 
            if tpos(1) > .39
                fsm = 'grasp';
            end

            % Move the tip to the next position (it moves along a line). 
            tpos(1) = tpos(1) + .01;
            yb.gripper_target.set_position(tpos,yb.arm_ref.id);

        case 'grasp'
             %% Grasp the object by closing the gripper on it.
            % Close the gripper. Please pay attention that it is not possible to determine the force to apply and 
            % object will sometimes slips from the gripper!
            yb.close_gripper;%res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            pause(2);
            
            % Disable IK; this is used at the next state to move the joints manually. 
            yb.setkinematicmode(0);
            fsm = 'backoff';
        case 'backoff'
            %% Go back to rest position.
            % Set each joint to their original angle. 
            yb.arm.set_state(startPose);
            
            % Get the gripper position and check whether it is at destination.
            tpos = yb.gripper.position(yb.arm_ref.id);
            if norm(tpos - gripperHome) < .02
                % Open the gripper. 
                yb.open_gripper; 
            end
            
            if norm(tpos - gripperHome) < .002
                fsm = 'finished';
            end
        case 'finished'
            %% Demo done: Break the loop.
            pause(3)
            break;
        otherwise
            error('Unknown state %s.',fsm);
    end
    
    
    % Update wheel velocities using the global values (whatever the state is).
    out = mo_ctrl.update(forwBackVel, leftRightVel, rotVel);
    yb.move(out(1), out(2), out(3));

    % Make sure that we do not go faster that the simulator (each iteration must take 50 ms). 
    elapsed = toc;
    timeleft = 0.05 - elapsed;
    if timeleft > 0
        pause(min(timeleft, .01));
    end
    
    prev_timestep = toc;
    
end
