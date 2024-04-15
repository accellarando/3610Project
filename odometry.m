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
MAX_SPEED		= 50; % if PID wants to go faster, bound it to this
MIN_SPEED		= 5; % if PID wants to go slower, set to 0

% Margin of acceptable error for PID control distance
DISTANCE_ERROR	= 0.5; % cm
ANGLE_ERROR		= 5; % degrees

% PID terms. Will definitely need tuning.
KpS = -1;
KiS = -0.5;
KdS = 0.007;

% global nanobot variable
odo_nb; % idk how scope works in matlab??

%% Init function
function [] = init_odometry(m_nanobot)
	global nb;
	nb = m_nanobot;
end

%% Encoder functions

% Get encoder counts
function [left, right] = get_encoders()
	global nb;
	left = nb.encoderRead(1).counts;
	right = nb.encoderRead(2).counts;
end

% Convert counts to distance
function [distance] = counts_to_distance(counts)
	distance = (counts / COUNTS_PER_REV) * (2 * pi * WHEEL_RADIUS); % revs * circumference
end

%% Motion functions

% Set motor speeds: assumes left is motor 1, right is 2
function [] = set_speeds(left, right)
	global nb;

	% clamp
	if abs(left) > MAX_SPEED
		left = MAX_SPEED * sign(left);
	elseif abs(left) < MIN_SPEED
		left = 0;
	end

	if abs(right) > MAX_SPEED
		right = MAX_SPEED * sign(right);
	elseif abs(right) < MIN_SPEED
		right = 0;
	end

	nb.setMotor(1, left);
	nb.setMotor(2, right);
end

% PID control for straight shot distance
function[] = go_straight_distance(distance)
	global nb;

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
	while ( !( (distTravelledLeft > distance - DISTANCE_ERROR) && 
					(distTravelledLeft < distance + DISTANCE_ERROR) ) ||
			!( (distTravelledRight > distance - DISTANCE_ERROR) &&
					(distTravelledRight < distance + DISTANCE_ERROR)))

		% timekeeping? idfk
		dt = toc - prevTime;
		prevTime = toc;

		% get encoder counts
		[left, right] = get_encoders();
		
		% convert to distance
		distTravelledLeft = counts_to_distance(left);
		distTravelledRight = counts_to_distance(right);

		% LEFT control
		errL = distTravelledLeft - distance;
		if(abs(errL) > DISTANCE_ERROR) 
			integralL = integralL + errL * dt;
			derivativeL = (errL - prevErrL) / dt;
			pidL = (KpS * errL) + (KiS * integralL) + (KdS * derivativeL);
			currSpeedL = currSpeedL + pidL;
			prevErrL = errL;
		else
			currSpeedL = 0;
		end


		% RIGHT control
		errR = distTravelledRight - distance;
		if(abs(errR) > DISTANCE_ERROR) 
			integralR = integralR + errR * dt;
			derivativeR = (errR - prevErrR) / dt;
			pidR = (KpS * errR) + (KiS * integralR) + (KdS * derivativeR);
			currSpeedR = currSpeedR + pidR;
			prevErrR = errR;
		else
			currSpeedR = 0;
		end

		set_speeds(currSpeedL, currSpeedR);

		% wait for next sample
		pause(1/SAMPLE_FREQ);
	end
end

% PID control for turn to angle, [-180, 180]
function [] = turn_degrees(angle)
	if(abs(angle) > 180)
		return;
	end

	% ????

	% variables
	turnRadius = WHEEL_DISTANCE / 2;
	turnCircumference = 2 * pi * turnRadius;
	turnDistance = (angle / 360) * turnCircumference;
	tolerance = (ANGLE_ERROR / 360) * turnCircumference;
	
	% what the... FUCK is going on
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
	while ( !( (distTravelledLeft > goalLeft - tolerance) && 
					(distTravelledLeft < goalLeft + tolerance) ) ||
			!( (distTravelledRight > goalRight - tolerance) &&
					(distTravelledRight < goalRight + tolerance)))

		dt = toc - prevTime;
		prevTime = toc;

		% get encoder counts
		[left, right] = get_encoders();
		
		% convert to distance
		distTravelledLeft = counts_to_distance(left);
		distTravelledRight = counts_to_distance(right);

		% LEFT control
		errL = distTravelledLeft - goalLeft;
		if(abs(errL) > tolerance) 
			integralL = integralL + errL * dt;
			derivativeL = (errL - prevErrL) / dt;
			pidL = (KpS * errL) + (KiS * integralL) + (KdS * derivativeL);
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
			pidR = (KpS * errR) + (KiS * integralR) + (KdS * derivativeR);
			currSpeedR = currSpeedR + pidR;
			prevErrR = errR;
		else
			currSpeedR = 0;
		end

		set_speeds(currSpeedL, currSpeedR);

		% wait for next sample
		pause(1/SAMPLE_FREQ);
	end
end



