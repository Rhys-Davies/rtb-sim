
% This script will find the rescue point and two obstacles in the reference
% frame of RescueRef. This is really just a "graphical" way of generating
% the problem. Move RescuePoint, RescueRef and the 5 obstacles to the 
% desired position in the simulation, pick which two obstacles to
% use, and run the script.


s = VREP();

obs1 = s.entity('29');
obs2 = s.entity('38');
resRef = s.entity('RescueRef');
resPnt = s.entity('RescuePoint');

obs1pos = obs1.position(resRef)
obs2pos = obs2.position(resRef)
rescue = resPnt.position(resRef)


