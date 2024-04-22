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
		COUNTS_PER_REV	= 1440.0; % Encoder counts per revolution
		WHEEL_RADIUS	= 3.5;	% cm
		WHEEL_DISTANCE	= 15.0;	% cm

		SAMPLE_FREQ		= 500.0; % Hz; may need to adjust if you get aliasing

		% Abs motor speed limits: [0, 100]. May need tuning.
		MAX_SPEED		= 12.0; % if PID wants to go faster, bound it to this
		MIN_SPEED		= 5.0; % if PID wants to go slower, set to 0

		% Margin of acceptable error for PID control distance
		DISTANCE_ERROR	= 0.75; % cm
		ANGLE_ERROR		= 7.5; % degrees

		% PID terms. Will definitely need tuning.
		KpS = -0.06;
		KiS = 0; %-0.003;
		KdS = 0;
		%KiS = -0.003;
		%KdS = 0.0001; %0.007;

	end
	methods
		function obj = odometry(val)
			obj.nb = val;
			obj.nb.initReflectance();
        end

		function val = reflectanceRead(m)
			val = m.nb.reflectanceRead();
		end

		% autonomous color task
        function [] = do_color_task(m)
            % Read color under sensor
			[r,g,b] = m.get_color();

			% Figure out if it's red, turn accordingly
			isRed = r > b;
			fprintf("Red: %d\n", isRed);
			
			m.set_speeds(-8,-10);
			pause(0.25);
			fprintf("off the pad\n");
			if(isRed)
				m.turn_degrees(45);
			else
				m.turn_degrees(-45);
			end
			fprintf("turned\n");

			[r,g,b] = m.get_color()

			if(isRed)
				while(r < 160)
					[r,g,b] = m.get_color()
					if(r > 160)
						m.set_speeds(0,0);
						break;
					end
					m.set_speeds(-8,-10);
					pause(0.25);
				end
			else
				while(b < 100)
					[r,g,b] = m.get_color()
					if(b > 100)
						m.set_speeds(0,0);
						break;
					end
					m.set_speeds(-8,-10);
					pause(0.25);
				end
			end

			input("Target reached?? Return?");
			m.turn_degrees(179);
			m.set_speeds(-8,-10);
			pause(0.25);
			while(1)
				[r,g,b] = m.get_color()
				if(r > 160 || b > 100)
					m.set_speeds(0,0);
					break;
				end
				m.set_speeds(-8,-10);
				pause(0.25);
			end
			fprintf("done");
        end

		% Get RGB under color sensor
		function [r,g,b] = get_color(m)
            m.nb.initColor();
			value = m.nb.colorRead();
			r = value.red;
			g = value.green;
			b = value.blue;
		end

		% Get encoder counts
		function [left, right] = get_encoders(m)
			left = -1 * m.nb.encoderRead(1).counts; % left is counting backwards for some reason idk
			right = m.nb.encoderRead(2).counts;
		end

		% Convert counts to distance
		function [distance] = counts_to_distance(m,counts)
			distance = (counts / m.COUNTS_PER_REV) * (2.0 * pi * m.WHEEL_RADIUS); % revs * circumference
		end


		% Set motor speeds: assumes left is motor 1, right is 2
		function [] = set_speeds(m, left, right)

			% clamp
			if abs(left) > m.MAX_SPEED
				left = m.MAX_SPEED * sign(left);
			elseif abs(left) < m.MIN_SPEED
				left = m.MIN_SPEED * sign(left);
			end

			if abs(right) > m.MAX_SPEED
				right = m.MAX_SPEED * sign(right);
			elseif abs(right) < m.MIN_SPEED
				right = m.MIN_SPEED * sign(right);
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
			while ( ~( (distTravelledLeft > distance - m.DISTANCE_ERROR) && (distTravelledLeft < distance + m.DISTANCE_ERROR) ) || ~( (distTravelledRight > distance - m.DISTANCE_ERROR) &&	(distTravelledRight < distance + m.DISTANCE_ERROR)))

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
				% currSpeedL, currSpeedR
				m.set_speeds(currSpeedL, currSpeedR);

				if(currSpeedL < m.MIN_SPEED && currSpeedR < m.MIN_SPEED)
					return;;
				end

				% wait for next sample
				pause(1/m.SAMPLE_FREQ);
			end
		end

		% PID control for turn to angle, [-180, 180]
		function [] = turn_degrees(m,angle)
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




