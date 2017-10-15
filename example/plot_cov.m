
function plot_cov(muX,SigmaX,number_of_sigmas)
SigmaX = SigmaX(1:2,1:2); 
muX = muX(1:2);
if(~any(diag(SigmaX)==0))
    [V,D] = eig(SigmaX);
    y = number_of_sigmas*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)];
    el = V*sqrtm(D)*y;
    el = [el el(:,1)]+repmat(muX,1,size(el,2)+1);
    line(el(1,:),el(2,:));
end;

% function plot_cov(muX,SigmaX,number_of_sigmas)
% SigmaX = SigmaX(1:2,1:2); % Get the top 2x2 matrix 
% muX = muX(1:2); % Get only x and y
% if(~any(diag(SigmaX)==0)) % If all the diagnals of sigma are non-zero
%     [V,D] = eig(SigmaX); % Get the two eigenvalues
%     y = number_of_sigmas*[cos(0:0.1:2*pi);sin(0:0.1:2*pi)]; %
%     el = V*sqrtm(D)*y; % V * principal square root of the square matrix D * y
%     el = [el el(:,1)]+repmat(muX,1,size(el,2)+1); % 
%     line(el(1,:),el(2,:)); % Draw the ellipse
% end