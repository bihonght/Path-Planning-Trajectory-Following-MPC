%
% function nu = wrap(alpha);
%
% 
% limits angles to -pi +pi
%
%

function nu = wrap(alpha)

nu = wrapToPi(alpha);

% nu = mod(alpha + pi, 2 * pi) - pi;
	% while (nu > pi)
	% 	nu = nu - 2 * pi;
	% end;
    % 
	% while (nu < -pi)
	% 	nu = nu + 2 * pi;
	% end;
