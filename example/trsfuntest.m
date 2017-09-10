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


%%
%
% Weird driving behaviour is because of opmodes. Not in streaming mode, so
% delay time makes controller react slowly.
% Joint position. Object position + orientation. Reading sensors.
% Worth noting: Calls using opmode_streaming do not return data from the
% buffer. All subsequent calls must use simx_opmode_buffer. To stop
% streaming a call must be made using simx_opmode_discontinue.
%
% Will need to implement an argument for overriding opmode to make pausing
% communications, sending a number of commands, then unpausing possible.
%
% Ideally, setters should use opmode_oneshot.
% Getters that are not streaming should use oneshot_wait
%
%%

sim = VREP(); % Default is to attempt connection to localhost
yb = sim.youbot('youBot'); % TRS youBot is a modified version of the stock VREP one.
sim.startSim();

timestep = 0.05;

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

gripperHome = yb.gripper.position(yb.arm_ref.id);

% Initialise state machine
fsm = 'rotate';

%% Start the Demo
while true
    tic
    
    youbotPos = yb.ref.position;
    youbotEuler = yb.ref.orientation;
    

    if plotData
        % Read data from the Hokuyo
        [pts, contacts] = yb.hokuyo.scan;
        
        s1 = yb.hokuyo.h1pos;
        s2 = yb.hokuyo.h2pos;
        
        % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo.
        in = inpolygon(X, Y, [s1(1), pts(1, :), s2(1)], [s1(2), pts(2, :), s2(2)]);
                    
        % Plot those points. Green dots: the visible area for the Hokuyo. Red starts: the obstacles. Red lines: the
        % visibility range from the Hokuyo sensor. 
        % The youBot is indicated with two dots: the blue one corresponds to the rear, the red one to the Hokuyo
        % sensor position. 
        subplot(211)
        plot(X(in), Y(in), '.g', pts(1, contacts), pts(2, contacts), '*r',...
             [s1(1), pts(1, :), s2(1)], [s1(2), pts(2, :), s2(2)], 'r',...
             0, 0, 'ob', s1(1), s1(2), 'or', s2(1), s2(2), 'or');
        axis([-5.5, 5.5, -5.5, 2.5]);
        axis equal;
        drawnow;
                      
    end
    
    
    angl = -pi/2;

%% Apply the state machine.

    switch (fsm)
        case 'rotate' 
            %% First, rotate the robot to go to one table.             % The rotation velocity depends on the difference between the current angle and the target. 
            
            rotVel = angdiff(angl, youbotEuler(3));

            % When the rotation is done (with a sufficiently high precision), move on to the next state. 
            if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                    (abs(angdiff(prevOri, youbotEuler(3))) < .01 / 180 * pi)
                rotVel = 0;
                fsm = 'drive';
            end

            prevOri = youbotEuler(3);   
        case 'drive'
            %% Then, make it move straight ahead until it reaches the table. 
            % The further the robot, the faster it drives. (Only check for the first dimension.)
            forwBackVel = -(youbotPos(1) + 3.167);

            % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
            % a specific location before moving on to the next state.
            if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevLoc) < .001)
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
            pts = yb.rgbdcamera.point_cloud;
            
             pts = pts(1:3, pts(4, :) < 1);

            if plotData
                subplot(223)
                plot3(pts(1, :), pts(3, :), pts(2, :), '*');
                axis equal;
                view([-169 -46]);
            end
            
            image = yb.rgbdcamera.image;
            
            if plotData
                subplot(224)
                imshow(image);
                drawnow;
            end

            % Next state. 
            fsm = 'extend';
            
        case 'extend'        
            %% TODO Move the arm to face the object.
            % Get the arm position. 
            
            
            tpos = yb.gripper.position(yb.arm_ref.id); % ptip rl2 armRef youBot_ref
            %youBot_gripperPositionTip
            % If the arm has reached the wanted position, move on to the next state. 
            
            if norm(tpos - [0.3259 -0.0010 0.2951]) < .002
                
                % Set the inverse kinematics (IK) mode to position AND orientation. 
                yb.setkinematicmode(2);
                
                fsm = 'reachout';
            end
        case 'reachout'
            %% TODO    %% Move the gripper tip along a line so that it faces the object with the right angle.
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
             %% TODO %% Grasp the object by closing the gripper on it.
            % Close the gripper. Please pay attention that it is not possible to determine the force to apply and 
            % object will sometimes slips from the gripper!
            yb.close_gripper;%res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            pause(2);
            
            % Disable IK; this is used at the next state to move the joints manually. 
            yb.setkinematicmode(0);
            fsm = 'backoff';
        case 'backoff'
            %% TODO %% Go back to rest position.
            % Set each joint to their original angle. 
            yb.arm.set_state(startPose);
            
            % Get the gripper position and check whether it is at destination.
            tpos = yb.gripper.position(yb.arm_ref.id); % (referenced to 'youBot_ref') 'youBot_gripperPositionTip'
            if norm(tpos - gripperHome) < .02
                % Open the gripper. 
                yb.open_gripper; % res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
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
    yb.drive(forwBackVel, leftRightVel, rotVel);

    % Make sure that we do not go faster that the simulator (each iteration must take 50 ms). 
    elapsed = toc;
    timeleft = timestep - elapsed;
    if timeleft > 0
        pause(min(timeleft, .01));
    end

end

clear all