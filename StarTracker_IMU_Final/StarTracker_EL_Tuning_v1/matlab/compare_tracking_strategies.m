function results = compare_tracking_strategies()
% Compare StarTracker tracking strategies with REAL measured IMU noise.
%  A  current Pi code: EL IMU PID (hysteresis .135/.150), AZ 0.03 deg bang-bang
%  B  sidereal feedforward on both axes + same feedback, feedback always on
%  C  feedforward + feedback, EL IMU feedback frozen while exposing
% Noise levels from real tuning logs (Downloads/StarTracker_tuning):
%  raw gravity tilt std ~0.12 deg, gyro std ~0.06 deg/s (stationary run).
% Blur = largest star displacement on the sensor within one 10 s exposure.

strategies = {'A_current', 'B_ff_feedback', 'C_ff_freeze'};
responses = [0.85 0.40];       % motor response: design guess / measured pulse ratio
seeds = 1:20;
pixel = 20.1/3600;             % deg per pixel (plate solve 2026-11-06)
rows = {};
for r = responses
    for s = 1:numel(strategies)
        blur = []; offs = []; settle = []; timeouts = 0;
        for seed = seeds
            out = runSequence(strategies{s}, r, seed);
            blur = [blur; out.blur]; offs = [offs; out.offset]; %#ok<AGROW>
            settle = [settle; out.settle]; timeouts = timeouts + out.timeouts; %#ok<AGROW>
        end
        rows(end+1,:) = {strategies{s}, r, median(blur)/pixel, prctile(blur,90)/pixel, ...
            max(blur)/pixel, median(offs)/pixel, median(settle), timeouts}; %#ok<AGROW>
    end
end
results = cell2table(rows, 'VariableNames', {'strategy','motor_response', ...
    'blur_med_px','blur_p90_px','blur_max_px','offset_med_px','settle_med_s','settle_timeouts'});
disp(results);
end

function out = runSequence(strategy, response, seed)
rng(seed);
dt = 0.05; step = 0.9/32/27;
lat = 37.6; dec = 54.87; ha0 = -30 + 60*rand;   % target east..west of meridian
sidereal = 360/86164.0905;                      % deg/s
exposure = 10; solve = 8; frames = 6; settleTimeout = 30; settleSamples = 5;
ff = strategy(1) ~= 'A'; freeze = strategy(1) == 'C';

% PID settings from Pi config.py
P = struct('kp',.315,'ki',.005,'kd',.125,'vmax',.05,'amax',.20,'tol',.150,'stop',.135);
pid = pidReset();
imu = struct('angle',[],'bias',.12+0.02*randn,'p00',.09,'p01',0,'p11',.01);
gyroBiasTrue = .12; rawStd = .12; gyroStd = .06;

[alt0, az0] = altaz(ha0, dec, lat);
altTrue = alt0 + 0.05*randn; azTrue = az0 + 0.05*randn;
imu.angle = altTrue + 0.03*randn;           % filtered angle at reference time
imuOffset = alt0 - imu.angle;               % IMUTrackingController.reference()
azEst = az0;                                % step estimate synced at start
remainder = 0; ffAlt = 0; ffAz = 0;
tNow = 0; prevAltTrue = altTrue;
[prevTA, prevTZ] = altaz(ha0, dec, lat);

blur = []; offset = []; settle = []; timeouts = 0;
phase = 'expose'; phaseT = 0; frame = 1; streak = 0; track = [];
while frame <= frames
    tNow = tNow + dt; phaseT = phaseT + dt;
    [tA, tZ] = altaz(ha0 + sidereal*tNow, dec, lat);
    % --- IMU read of TRUE axis + Kalman (as controllers/angle_kalman.py)
    gyro = (altTrue - prevAltTrue)/dt + gyroBiasTrue + gyroStd*randn;
    z = altTrue + rawStd*randn; prevAltTrue = altTrue;
    imu = kalman(imu, z, gyro, dt);
    altEst = imu.angle + imuOffset;
    % --- control
    exposing = strcmp(phase,'expose');
    eAlt = tA - altEst; eAz = wrap(tZ - azEst);
    sAlt = 0; sAz = 0;
    if ff
        ffAlt = ffAlt + (tA - prevTA)/step; ffAz = ffAz + wrap(tZ - prevTZ)/step;
        fa = trunc(ffAlt); ffAlt = ffAlt - fa; fz = trunc(ffAz); ffAz = ffAz - fz;
        sAlt = fa; sAz = fz;
    end
    if ~(freeze && exposing)
        [pid, v] = pidUpdate(pid, eAlt, altEst, dt, P);
        if pid.stopped, remainder = 0; end
        d = v*dt/step + remainder; n = clamp(trunc(d), -5, 5); remainder = d - n;
        sAlt = sAlt + n;
        if abs(eAz) >= .03, sAz = sAz + sign(eAz)*min(round(abs(eAz)/step), 5); end
    end
    sAlt = clamp(sAlt,-5,5); sAz = clamp(sAz,-5,5);
    altTrue = altTrue + sAlt*step*response;
    azTrue = azTrue + sAz*step*response;
    azEst = azEst + sAz*step;
    prevTA = tA; prevTZ = tZ;
    % --- star position on sensor (deg), relative to target
    dAlt = tA - altTrue; dAz = wrap(tZ - azTrue)*cosd(tA);
    switch phase
        case 'expose'
            track(end+1,:) = [dAlt dAz]; %#ok<AGROW>
            if phaseT >= exposure
                b = max(pdist2local(track)); blur(end+1,1) = b; %#ok<AGROW>
                mid = track(round(end/2),:); offset(end+1,1) = hypot(mid(1),mid(2)); %#ok<AGROW>
                % plate solve of mid-exposure TRUE pointing (+ ~1 px noise)
                corrAlt = mid(1) + .005*randn; corrAz = wrap(mid(2)/cosd(tA)) + .005*randn;
                track = []; frame = frame + 1; phase = 'solve'; phaseT = 0;
                if frame > frames, break; end
            end
        case 'solve'
            if phaseT >= solve
                % apply_position_offset(-alt_error, -az_error)
                imuOffset = imuOffset - corrAlt; azEst = azEst - corrAz;
                pid = pidReset(); remainder = 0;
                phase = 'settle'; phaseT = 0; streak = 0;
            end
        case 'settle'
            eA = tA - (imu.angle + imuOffset); eZ = wrap(tZ - azEst);
            if abs(eA) <= P.tol && abs(eZ) <= .03, streak = streak + 1; else, streak = 0; end
            if streak >= settleSamples || phaseT >= settleTimeout
                settle(end+1,1) = phaseT; timeouts = timeouts + (streak < settleSamples); %#ok<AGROW>
                phase = 'expose'; phaseT = 0;
                if freeze, pid = pidReset(); remainder = 0; end
            end
    end
end
out = struct('blur',blur,'offset',offset,'settle',settle,'timeouts',timeouts);
end

function [alt, az] = altaz(haDeg, dec, lat)
h = haDeg; alt = asind(sind(dec)*sind(lat) + cosd(dec)*cosd(lat)*cosd(h));
az = mod(atan2d(-sind(h)*cosd(dec), sind(dec)*cosd(lat) - cosd(dec)*sind(lat)*cosd(h)), 360);
end

function pid = pidReset()
pid = struct('integral',0,'prev',NaN,'speed',0,'stopped',true);
end

function [pid, speed] = pidUpdate(pid, err, meas, dt, P)
if isnan(pid.prev), delta = 0; else, delta = meas - pid.prev; end
pid.prev = meas;
if pid.stopped, thr = P.tol; else, thr = P.stop; end
pid.stopped = abs(err) <= thr;
if pid.stopped, pid.integral = 0; pid.speed = 0; speed = 0; return; end
cand = pid.integral + err*dt;
raw = P.kp*err + P.ki*cand - P.kd*delta/dt;
speed = clamp(clamp(raw, pid.speed - P.amax*dt, pid.speed + P.amax*dt), -P.vmax, P.vmax);
if raw == speed || (raw - speed)*err < 0, pid.integral = cand; end
pid.speed = speed;
end

function s = kalman(s, z, gyro, dt)
q1 = .01; q2 = .0001; r = .09;
if isempty(s.angle), s.angle = z; return; end
pred = s.angle + dt*(gyro - s.bias);
a = s.p00 - 2*dt*s.p01 + dt^2*s.p11 + q1*dt + q2*dt^3/3;
b = s.p01 - dt*s.p11 - q2*dt^2/2; c = s.p11 + q2*dt;
inn = wrap(z - pred); k0 = a/(a+r); k1 = b/(a+r);
s.angle = pred + k0*inn; s.bias = s.bias + k1*inn; u = 1 - k0;
s.p00 = u^2*a + k0^2*r; s.p01 = u*(b - k1*a) + k0*k1*r; s.p11 = c - 2*k1*b + k1^2*(a + r);
end

function d = pdist2local(x)
d = 0;
for i = 1:size(x,1)
    d = max(d, max(hypot(x(i,1) - x(:,1), x(i,2) - x(:,2))));
end
end

function y = wrap(x), y = mod(x + 180, 360) - 180; end
function y = clamp(x, lo, hi), y = max(lo, min(hi, x)); end
function y = trunc(x), y = fix(x); end
