function Bounds = reqr_bounds(Stm, Reqr)
% REQR_BOUNDS  Bounds of the step responses, imposed by the statement.
%
% Arguments:
%   Stm  (struct) -- Project statement data
%   Reqr (struct) -- Requirements estimation
% Return:
%   Bounds (struct) -- 

Bounds.Roll  = check_roll(Stm);
Bounds.Pitch = check_pitch(Stm);
Bounds.Yaw   = check_yaw(Stm, Reqr);
end

%% Roll bounds

function Bounds = check_roll(Stm)

ti = 0;
ts = Stm.Roll.settlingTime;
tf = Stm.Roll.settlingTime * 2;
angleOvershoot     = (1+Stm.Roll.maxOvershoot)   * Stm.Roll.angle;
angleSettlingUpper = (1+Stm.Roll.settlingRtol/2) * Stm.Roll.angle;
angleSettlingLower = (1-Stm.Roll.settlingRtol/2) * Stm.Roll.angle;

Bounds.Upper.time  = [ti, ts, ts, tf];
Bounds.Upper.angle = [angleOvershoot, angleOvershoot, angleSettlingUpper, angleSettlingUpper];
Bounds.Lower.time  = [ts, ts, tf];
Bounds.Lower.angle = [0, angleSettlingLower, angleSettlingLower];

Bounds.check = @check_bounds;

	function isInsideBounds = check_bounds(evoTime, evoAngle)
		before_ts = evoTime < ts;
		check_upper_before_ts = all(evoAngle(before_ts) <= angleOvershoot);

		after_ts = ~before_ts;
		check_upper_after_ts = all(evoAngle(after_ts) <= angleSettlingUpper);
		check_lower_after_ts = all(evoAngle(after_ts) >= angleSettlingLower);

		isInsideBounds = check_upper_before_ts && ...
		                 check_upper_after_ts  && ...
						 check_lower_after_ts;
	end
end

%% Pitch bounds

function Bounds = check_pitch(Stm)

ti = 0;
ts = Stm.Pitch.settlingTime;
tf = Stm.Pitch.settlingTime * 2;
angleOvershoot     = (1+Stm.Pitch.maxOvershoot) * Stm.Pitch.angle;
angleSettlingUpper = Stm.Pitch.angle + Stm.Pitch.settlingAngle/2;
angleSettlingLower = Stm.Pitch.angle - Stm.Pitch.settlingAngle/2;

Bounds.Upper.time  = [ti, ts, ts, tf];
Bounds.Upper.angle = [angleOvershoot, angleOvershoot, angleSettlingUpper, angleSettlingUpper];
Bounds.Lower.time  = [ts, ts, tf];
Bounds.Lower.angle = [0, angleSettlingLower, angleSettlingLower];

Bounds.check = @check_bounds;

	function isInsideBounds = check_bounds(evoTime, evoAngle)
		before_ts = evoTime < ts;
		check_upper_before_ts = all(evoAngle(before_ts) <= angleOvershoot);

		after_ts = ~before_ts;
		check_upper_after_ts = all(evoAngle(after_ts) <= angleSettlingUpper);
		check_lower_after_ts = all(evoAngle(after_ts) >= angleSettlingLower);

		isInsideBounds = check_upper_before_ts && ...
		                 check_upper_after_ts  && ...
						 check_lower_after_ts;
	end
end

%% Yaw bounds

function Bounds = check_yaw(Stm, Reqr)

ts = Stm.Yaw.settlingTime;
tf = Stm.Yaw.settlingTime * 2;
angleSettlingUpper = +Stm.Yaw.settlingRtol/2 * Reqr.YawEvo.devAngle;
angleSettlingLower = -Stm.Yaw.settlingRtol/2 * Reqr.YawEvo.devAngle;

Bounds.Upper.time  = [ts, tf];
Bounds.Upper.angle = [angleSettlingUpper, angleSettlingUpper];
Bounds.Lower.time  = [ts, tf];
Bounds.Lower.angle = [angleSettlingLower, angleSettlingLower];

Bounds.check = @check_bounds;

	function isInsideBounds = check_bounds(evoTime, evoAngle)
		after_ts = evoTime >= ts;
		check_upper_after_ts = all(evoAngle(after_ts) <= angleSettlingUpper);
		check_lower_after_ts = all(evoAngle(after_ts) >= angleSettlingLower);

		isInsideBounds = check_upper_after_ts && check_lower_after_ts;
	end
end
