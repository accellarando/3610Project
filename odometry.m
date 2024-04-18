

classdef odometry
	properties
		% properties
		nb
        %%%%%%%%%%
% ECE 3610
% Final Project
%
% Odometry Functions!
%%%%%%%%%%

%% Robot Parameters
COUNTS_PER_REV	= 1440; % Encoder counts per revolution
WHEEL_RADIUS	= 3.5;	% cm
WHEEL_DISTANCE	= 15;	% cm

SAMPLE_FREQ		= 1000; % Hz; may need to adjust if you get aliasing

% Abs motor speed limits: [0, 100]. May need tuning.
MAX_SPEED		= 20; % if PID wants to go faster, bound it to this
MIN_SPEED		= 5; % if PID wants to go slower, set to 0

% Margin of acceptable error for PID control distance
DISTANCE_ERROR	= 0.5; % cm
ANGLE_ERROR		= 5; % degrees

% PID terms. Will definitely need tuning.
KpS = -1;
KiS = -0.5;
KdS = 0.007;

	end
	methods
      function obj = odometry(val)
         if nargin > 0
            obj.nb = val;
         end
      end
		% Get encoder counts
		function [left, right] = get_encoders(m)
			left = m.nb.encoderRead(1).counts;
			right = m.nb.encoderRead(2).counts;
		end

		% Convert counts to distance
		function [distance] = counts_to_distance(m,counts)
			distance = (counts / m.COUNTS_PER_REV) * (2 * pi * m.WHEEL_RADIUS); % revs * circumference
		end


		% Set motor speeds: assumes left is motor 1, right is 2
		function [] = set_speeds(m, left, right)

			% clamp
			if abs(left) > m.MAX_SPEED
				left = m.MAX_SPEED * sign(left);
			elseif abs(left) < m.MIN_SPEED
				left = 0;
			end

			if abs(right) > m.MAX_SPEED
				right = m.MAX_SPEED * sign(right);
			elseif abs(right) < m.MIN_SPEED
				right = 0;
			end

			m.nb.setMotor(1, left);
			m.nb.setMotor(2, right);
		end

		% PID control for straight shot distance
		function[] = go_straight_distance(m,distance)

			distTravelledLeft = 0;
			distTravelledRight = 0;

			% PID control loop
			tic;
			prevTime = 0;
			prevErrL = 0;
			prevErrR = 0;
			integralL = 0;
			derivativeL = 0;
			integralR = 0;
			derivativeR = 0;
			currSpeedL = 0;
			currSpeedR = 0;
			while ( ~( (distTravelledLeft > distance - m.DISTANCE_ERROR) && 				(distTravelledLeft < distance + m.DISTANCE_ERROR) ) ||				~( (distTravelledRight > distance - m.DISTANCE_ERROR) &&				(distTravelledRight < distance + m.DISTANCE_ERROR)))

				% timekeeping? idfk
				dt = toc - prevTime;
				prevTime = toc;

				% get encoder counts
				[left, right] = m.get_encoders();

				% convert to distance
				distTravelledLeft = distTravelledLeft + m.counts_to_distance(left);
				distTravelledRight = distTravelledRight + m.counts_to_distance(right);

				% LEFT control
				errL = distTravelledLeft - distance;
				if(abs(errL) > m.DISTANCE_ERROR) 
					integralL = integralL + errL * dt;
					derivativeL = (errL - prevErrL) / dt;
					pidL = (m.KpS * errL) + (m.KiS * integralL) + (m.KdS * derivativeL);
					currSpeedL = currSpeedL + pidL;
					prevErrL = errL;
				else
					currSpeedL = 0;
				end


				% RIGHT control
				errR = distTravelledRight - distance;
				if(abs(errR) > m.DISTANCE_ERROR) 
					integralR = integralR + errR * dt;
					derivativeR = (errR - prevErrR) / dt;
					pidR = (m.KpS * errR) + (m.KiS * integralR) + (m.KdS * derivativeR);
					currSpeedR = currSpeedR + pidR;
					prevErrR = errR;
				else
					currSpeedR = 0;
				end

				% Also takes care of clamping
				m.set_speeds(currSpeedL, currSpeedR);

				% wait for next sample
				pause(1/m.SAMPLE_FREQ);
			end
		end

		% PID control for turn to angle, [-180, 180]
		function [] = turn_degrees(angle)
			if(abs(angle) > 180)
				return;
			end

			% ????

			% variables
			turnRadius = m.WHEEL_DISTANCE / 2;
			turnCircumference = 2 * pi * turnRadius;
			turnDistance = (angle / 360) * turnCircumference;
			tolerance = (m.ANGLE_ERROR / 360) * turnCircumference;

			% PID control loop
			tic;
			prevTime = 0;
			prevErrL = 0;
			prevErrR = 0;
			integralL = 0;
			derivativeL = 0;
			integralR = 0;
			derivativeR = 0;
			currSpeedL = 0;
			currSpeedR = 0;
			goalLeft = turnDistance;
			goalRight = -turnDistance;
			distTravelledLeft = 0;
			distTravelledRight = 0;
			while ( ~( (distTravelledLeft > goalLeft - tolerance) && 				(distTravelledLeft < goalLeft + tolerance) ) ||				~( (distTravelledRight > goalRight - tolerance) &&				(distTravelledRight < goalRight + tolerance)))

				dt = toc - prevTime;
				prevTime = toc;

				% get encoder counts
				[left, right] = m.get_encoders();

				% convert to distance
				distTravelledLeft = distTravelledLeft + m.counts_to_distance(left);
				distTravelledRight = distTravelledRight + m.counts_to_distance(right);

				% LEFT control
				errL = distTravelledLeft - goalLeft;
				if(abs(errL) > tolerance) 
					integralL = integralL + errL * dt;
					derivativeL = (errL - prevErrL) / dt;
					pidL = (m.KpS * errL) + (m.KiS * integralL) + (m.KdS * derivativeL);
					currSpeedL = currSpeedL + pidL;
					prevErrL = errL;
				else
					currSpeedL = 0;
				end


				% RIGHT control
				errR = distTravelledRight - goalRight;
				if(abs(errR) > tolerance) 
					integralR = integralR + errR * dt;
					derivativeR = (errR - prevErrR) / dt;
					pidR = (m.KpS * errR) + (m.KiS * integralR) + (m.KdS * derivativeR);
					currSpeedR = currSpeedR + pidR;
					prevErrR = errR;
				else
					currSpeedR = 0;
				end

				% Also takes care of clamping
				m.set_speeds(currSpeedL, currSpeedR);

				% wait for next sample
				pause(1/m.SAMPLE_FREQ);
			end
		end
	end
end




