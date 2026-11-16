% Open this file in MATLAB and press Run. Set STARTRACKER_PI_PROJECT to use
% another copy of the Pi project. The script runs without hardware.
scriptDir = fileparts(mfilename('fullpath'));
project = string(getenv('STARTRACKER_PI_PROJECT'));
if project == ""
    project = string(fileparts(scriptDir));
end
runPiTests(project, scriptDir);

function runPiTests(project, scriptDir)
    p = loadSettings(fullfile(project, "config.py"));
    checkPidThresholds(p);
    assert(settleStreak(4, 0.1, 0.02, false, p) == 0);
    assert(settleStreak(4, 0.1, 0.04, true, p) == 0);
    assert(settleStreak(4, 0.16, 0.02, true, p) == 0);
    assert(settleStreak(4, 0.1, angularError(0.01, 359.99), true, p) == 5);

    [trace, phases] = stepResponse(p);
    assert(all(isfinite(phases.final_stop_s)), 'A step phase did not end stopped.');
    solverTimes = repmat(8, 1, p.frames - 1);
    altOffsets = repmat([0.30, -0.30], 1, ceil((p.frames - 1)/2));
    [normal, nominalTimeline] = captureSequence(p, solverTimes, altOffsets);
    assert(normal.status == "completed" && normal.frames_saved == p.frames);
    assert(normal.exposure_s == p.frames * p.exposure);
    assert(sum(nominalTimeline.stage == "settle") == p.frames - 1);
    slowSolver = solverTimes;
    slowSolver(1) = p.solveTimeout + 1;
    late = captureSequence(p, slowSolver, altOffsets);
    assert(late.status == "solve_timeout" && late.frames_saved == 1);
    largeCorrection = altOffsets;
    largeCorrection(1) = 1.6;
    [unsettled, timeoutTimeline] = captureSequence(p, solverTimes, largeCorrection);
    assert(unsettled.status == "completed" && unsettled.frames_saved == p.frames);
    assert(unsettled.settle_timeouts >= 1);
    secondExposure = timeoutTimeline.start_s(timeoutTimeline.frame == 2 & timeoutTimeline.stage == "exposure");
    assert(abs(secondExposure - (p.exposure + solverTimes(1) + p.settleTimeout)) < 1e-8);
    scenarios = addvars(struct2table([normal; late; unsettled]), ...
        ["8 s solve and 0.30 deg correction"; "Solver over limit"; "1.60 deg first EL correction"], ...
        'Before', 1, 'NewVariableNames', 'scenario');

    out = fullfile(scriptDir, "startracker_pi_matlab_results");
    if ~isfolder(out)
        mkdir(out);
    end
    writetable(trace, fullfile(out, "control.csv"));
    writetable(phases, fullfile(out, "step_response.csv"));
    writetable(scenarios, fullfile(out, "capture_scenarios.csv"));
    writetable(nominalTimeline, fullfile(out, "capture_timeline.csv"));
    exportResponse(trace, nominalTimeline, p, fullfile(out, "response.png"));
    fprintf('PID edge tests passed. Kp=%g Ki=%g Kd=%g.\n', p.kp, p.ki, p.kd);
    disp(phases);
    disp(scenarios);
    fprintf('Results: %s\n', out);
end

function p = loadSettings(configFile)
    txt = fileread(configFile);
    names = {'dt','maxSteps','pulseDelay','minAlt','maxAlt','qAngle','qBias','rAngle', ...
        'initialBiasVariance','stallSeconds','stallCommand','stallMovement','azTolerance', ...
        'frames','exposure','solveTimeout','settleTimeout','settleSamples'};
    keys = {'TRACKING_INTERVAL','MAX_STEP_PER_UPDATE','MOTOR_PULSE_DELAY','MIN_ALTITUDE', ...
        'MAX_ALTITUDE','IMU_KALMAN_Q_ANGLE','IMU_KALMAN_Q_BIAS','IMU_KALMAN_R_ANGLE', ...
        'IMU_KALMAN_INITIAL_BIAS_VARIANCE','IMU_TRACKING_STALL_SECONDS', ...
        'IMU_TRACKING_STALL_COMMAND_DEG','IMU_TRACKING_STALL_MOVEMENT_DEG', ...
        'ANGLE_TOLERANCE','PIPELINED_FRAME_COUNT','PIPELINED_EXPOSURE_SECONDS', ...
        'PLATE_SOLVE_TIMEOUT','PIPELINED_SETTLE_TIMEOUT_SECONDS','PIPELINED_SETTLE_SAMPLES'};
    for i = 1:numel(keys)
        p.(names{i}) = readNumber(txt, ['(?m)^' keys{i} '\s*=\s*'], keys{i});
    end
    pid = regexp(txt, 'IMU_TRACKING_PID\s*=\s*dict\(([^)]*)\)', 'tokens', 'once');
    assert(~isempty(pid), 'Missing IMU_TRACKING_PID in config.py.');
    names = {'kp','ki','kd','speed','accel','tolerance','stopTolerance'};
    keys = {'kp','ki','kd','max_speed','max_acceleration','tolerance','stop_tolerance'};
    for i = 1:numel(keys)
        p.(names{i}) = readNumber(pid{1}, ['(?:^|[,\s])' keys{i} '\s*=\s*'], keys{i});
    end
    step = readNumber(txt, '(?m)^MOTOR_FULL_STEP_ANGLE\s*=\s*', 'MOTOR_FULL_STEP_ANGLE');
    microstep = readNumber(txt, '(?m)^MICROSTEP\s*=\s*', 'MICROSTEP');
    gear = readNumber(txt, '(?m)^GEAR_RATIO\s*=\s*', 'GEAR_RATIO');
    p.stepAngle = step / microstep / gear;
    for flag = ["IMU_TRACKING_ENABLED","PIPELINED_CAPTURE_ENABLED","PIPELINED_WAIT_FOR_SETTLE"]
        assert(~isempty(regexp(txt, "(?m)^" + flag + "\s*=\s*True(?:\s|$)", 'once')), ...
            'Expected %s=True in config.py.', flag);
    end
    assert(~isempty(regexp(txt, '(?m)^IMU_FILTER_MODE\s*=\s*"kalman"', 'once')), ...
        'This simulation expects Kalman EL tracking.');
    assert(p.stopTolerance <= p.tolerance && p.frames >= 2);
    p.motorResponse = 0.85;  % Synthetic response fraction from SimulatedMount.
end

function value = readNumber(source, prefix, name)
    match = regexp(source, [prefix '([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)'], 'tokens', 'once');
    assert(~isempty(match), 'Cannot read %s from config.py.', name);
    value = str2double(match{1});
end

function [trace, phases] = stepResponse(p)
    % Baseline 5 s, then three +1 deg / return cycles of 60 s per move.
    baseline = 1.46;
    targets = [baseline, repmat([baseline + 1, baseline], 1, 3)];
    durations = [5, repmat(60, 1, 6)];
    ticks = round(durations / p.dt);
    values = zeros(sum(ticks), 10);
    s = startAxis(baseline);
    tick = 0;
    for phase = 1:numel(targets)
        for j = 1:ticks(phase)
            t = tick * p.dt;
            tick = tick + 1;
            [s, row] = updateAxis(s, t, targets(phase), p);
            values(tick, :) = [t, phase-1, targets(phase), row];
        end
    end
    columns = {'time_s','phase','target_alt_deg','model_alt_deg','imu_alt_deg','imu_error_deg', ...
        'speed_command_deg_s','signed_steps','pid_stopped','fractional_steps'};
    trace = array2table(values, 'VariableNames', columns);
    summary = nan(6, 5);
    for phase = 1:6
        rows = trace(trace.phase == phase, :);
        beforeStop = find(rows.pid_stopped == 0, 1, 'last');
        firstStop = find(rows.pid_stopped == 1, 1);
        if ~isempty(firstStop)
            summary(phase, 1) = (firstStop-1) * p.dt;
        end
        if rows.pid_stopped(end)
            summary(phase, 2) = max([0, beforeStop]) * p.dt;
        end
        summary(phase, 3) = nnz(diff(rows.pid_stopped) == -1);
        summary(phase, 4:5) = [rows.imu_error_deg(end), rows.target_alt_deg(end) - rows.model_alt_deg(end)];
    end
    labels = reshape(["step_" + (1:3); "return_" + (1:3)], [], 1);
    phases = addvars(array2table(summary, 'VariableNames', {'first_stop_s','final_stop_s', ...
        'restarts','end_imu_error_deg','end_model_error_deg'}), labels, ...
        'Before', 1, 'NewVariableNames', 'phase');
end

function s = startAxis(alt)
    s = struct('actual',alt, 'previousActual',alt, 'raw',alt+7, 'offset',-7, ...
        'kalman',[], 'previousTime',NaN, 'az',359.98, 'watch',[]);
    s.pid = resetPid();
end

function pid = resetPid()
    pid = struct('previous',NaN, 'integral',0, 'speed',0, 'remainder',0, 'stopped',true);
end

function [s, row] = updateAxis(s, time, target, p)
    actual = s.actual;
    dt = p.dt;
    if isnan(s.previousTime)
        s.kalman = [s.raw, 0.12, p.rAngle, 0, p.initialBiasVariance];
    else
        dt = time - s.previousTime;
        gravity = actual + 7 + 0.02 * sin(7 * time);
        gyro = (actual - s.previousActual) / dt + 0.12 + 0.003 * sin(11 * time);
        s.raw = s.raw + angularError(gravity, s.raw);
        s.kalman = estimateAngle(s.kalman, s.raw, gyro, dt, p);
    end
    s.previousTime = time;
    s.previousActual = actual;
    estimate = s.kalman(1) + s.offset;
    errorAlt = target - estimate;
    [s.pid, steps] = updatePid(s.pid, errorAlt, estimate, dt, p);
    room = p.maxAlt - estimate;
    if steps <= 0
        room = estimate - p.minAlt;
    end
    steps = sign(steps) * min(abs(steps), floor(max(0, room) / p.stepAngle));
    if ~isempty(s.watch)
        if abs(estimate - s.watch(2)) >= p.stallMovement
            s.watch = [];
        elseif time - s.watch(1) >= p.stallSeconds
            assert(s.watch(3) < p.stallCommand, 'Simulated EL motor stalled.');
            s.watch = [];
        end
    end
    if steps ~= 0
        if isempty(s.watch)
            s.watch = [time, estimate, 0];
        end
        s.watch(3) = s.watch(3) + abs(steps) * p.stepAngle;
    end
    s.actual = actual + steps * p.stepAngle * p.motorResponse;
    row = [actual, estimate, errorAlt, s.pid.speed, steps, s.pid.stopped, s.pid.remainder];
end

function [pid, steps] = updatePid(pid, errorAlt, estimate, dt, p)
    delta = 0;
    if ~isnan(pid.previous)
        delta = estimate - pid.previous;
    end
    pid.previous = estimate;
    threshold = p.stopTolerance;
    if pid.stopped
        threshold = p.tolerance;
    end
    pid.stopped = abs(errorAlt) <= threshold;
    if pid.stopped
        pid.integral = 0;
        pid.speed = 0;
        pid.remainder = 0;
    else
        candidate = pid.integral + dt * errorAlt;
        raw = p.kp * errorAlt + p.ki * candidate - p.kd * delta / dt;
        limit = min([p.speed, p.maxSteps * p.stepAngle / dt, p.stepAngle / (2 * p.pulseDelay)]);
        change = p.accel * dt;
        pid.speed = clamp(clamp(raw, pid.speed-change, pid.speed+change), -limit, limit);
        if raw == pid.speed || (raw-pid.speed) * errorAlt < 0
            pid.integral = candidate;
        end
    end
    demand = pid.speed * dt / p.stepAngle + pid.remainder;
    steps = clamp(fix(demand), -p.maxSteps, p.maxSteps);
    pid.remainder = demand - steps;
end

function checkPidThresholds(p)
    low = p.stopTolerance;
    high = p.tolerance;
    middle = (low + high) / 2;
    for direction = [1, -1]
        pid = resetPid();
        errors = [high, high+0.001, high, middle, low, middle, high, high+0.001];
        stopped = [true, false, false, false, true, true, true, false];
        for i = 1:numel(errors)
            [pid, steps] = updatePid(pid, direction * errors(i), 0, p.dt, p);
            assert(pid.stopped == stopped(i), 'PID threshold transition differs from Pi.');
            if pid.stopped
                assert(pid.speed == 0 && pid.integral == 0 && pid.remainder == 0 && steps == 0);
            end
        end
    end
end

function [result, timeline] = captureSequence(p, solveTimes, altOffsets)
    % Model starts at the first exposure. Save/readout overhead is omitted.
    % Solver time and plate-derived offsets are scenario inputs.
    s = startAxis(40);
    [s, ~] = updateAxis(s, 0, 40, p);
    frames = 0;
    settleTimeouts = 0;
    times = zeros(3*p.frames - 2, 3);
    stages = strings(size(times, 1), 1);
    event = 0;
    tick = 0;
    status = "completed";
    for frame = 1:p.frames
        [s, tick, ~] = trackFor(s, tick, p.exposure, false, p);
        event = event + 1;
        times(event, :) = [frame, tick*p.dt-p.exposure, tick*p.dt];
        stages(event) = "exposure";
        frames = frame;  % Camera file saved before solving begins.
        if frame == p.frames
            break;
        end
        started = tick * p.dt;
        [s, tick, ~] = trackFor(s, tick, min(solveTimes(frame), p.solveTimeout), false, p);
        event = event + 1;
        times(event, :) = [frame, started, tick*p.dt];
        stages(event) = "solve";
        if solveTimes(frame) > p.solveTimeout
            status = "solve_timeout";
            break;
        end
        s.offset = s.offset - altOffsets(frame);
        s.az = mod(s.az - 0.06, 360);
        s.pid = resetPid();
        s.watch = [];
        started = tick * p.dt;
        [s, tick, stable] = trackFor(s, tick, p.settleTimeout, true, p);
        event = event + 1;
        times(event, :) = [frame, started, tick*p.dt];
        stages(event) = "settle";
        if ~stable
            settleTimeouts = settleTimeouts + 1;
        end
    end
    timeline = addvars(array2table(times(1:event,:), 'VariableNames', {'frame','start_s','end_s'}), ...
        stages(1:event), 'NewVariableNames', 'stage');
    result = struct('status', status, 'frames_saved', frames, 'exposure_s', frames*p.exposure, ...
        'solve_s', durationOf(timeline, "solve"), 'settle_s', durationOf(timeline, "settle"), ...
        'settle_timeouts', settleTimeouts, 'elapsed_s', tick*p.dt);
end

function duration = durationOf(timeline, stage)
    rows = timeline.stage == stage;
    duration = sum(timeline.end_s(rows) - timeline.start_s(rows));
end

function [s, tick, stable] = trackFor(s, tick, seconds, settling, p)
    targetAlt = 40;
    targetAz = 0.01;
    stable = false;
    streak = 0;
    for i = 1:ceil(seconds / p.dt)
        tick = tick + 1;
        [s, row] = updateAxis(s, tick*p.dt, targetAlt, p);
        azError = angularError(targetAz, s.az);
        if abs(azError) >= p.azTolerance
            steps = min(round(abs(azError)/p.stepAngle), p.maxSteps);
            s.az = mod(s.az + sign(azError)*steps*p.stepAngle, 360);
        end
        if settling
            streak = settleStreak(streak, row(3), angularError(targetAz, s.az), true, p);
            if streak >= p.settleSamples
                stable = true;
                return;
            end
        end
    end
end

function streak = settleStreak(streak, elError, azError, fresh, p)
    if fresh && abs(elError) <= p.tolerance && abs(azError) <= p.azTolerance
        streak = streak + 1;
    else
        streak = 0;
    end
end

function error = angularError(target, actual)
    error = mod(target - actual + 180, 360) - 180;
end

function y = clamp(x, lower, upper)
    y = max(lower, min(upper, x));
end

function state = estimateAngle(state, gravity, gyro, dt, p)
    % Joseph covariance update reused from the earlier MATLAB EL simulation.
    predicted = state(1) + dt * (gyro - state(2));
    a = state(3) - 2*dt*state(4) + dt^2*state(5) + p.qAngle*dt + p.qBias*dt^3/3;
    b = state(4) - dt*state(5) - p.qBias*dt^2/2;
    c = state(5) + p.qBias*dt;
    innovation = angularError(gravity, predicted);
    k0 = a / (a + p.rAngle);
    k1 = b / (a + p.rAngle);
    u = 1 - k0;
    state = [predicted + k0*innovation, state(2) + k1*innovation, ...
        u^2*a + k0^2*p.rAngle, u*(b-k1*a) + k0*k1*p.rAngle, ...
        c - 2*k1*b + k1^2*(a + p.rAngle)];
end

function exportResponse(trace, timeline, p, file)
    fig = figure('Name', 'StarTracker Pi MATLAB test', 'NumberTitle', 'off', ...
        'Color', 'white', 'Position', [100, 100, 1060, 740]);
    tabs = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    t = trace.time_s;
    a = nexttile(tabs);
    stairs(a, t, trace.target_alt_deg, 'k:', 'LineWidth', 1.3);
    hold(a, 'on');
    plot(a, t, trace.imu_alt_deg, 'LineWidth', 1.3);
    legend(a, 'Target', 'IMU estimate', 'Location', 'eastoutside');
    ylabel(a, 'Altitude (deg)');
    title(a, 'EL step and return test   |   simulated motor response 85%');
    b = nexttile(tabs);
    plot(b, t, trace.imu_error_deg, 'LineWidth', 1.3);
    hold(b, 'on');
    color = [0.50 0.30 0.11];
    restart = yline(b, p.tolerance, '--', 'Color', color);
    yline(b, -p.tolerance, '--', 'Color', color);
    stop = yline(b, p.stopTolerance, ':k');
    yline(b, -p.stopTolerance, ':k');
    legend(b, [restart, stop], 'Restart threshold', 'Stop threshold', 'Location', 'eastoutside');
    ylabel(b, 'EL error (deg)');
    xlabel(b, 'Step test time (s)');
    linkaxes([a, b], 'x');
    grid(a, 'on');
    grid(b, 'on');
    c = nexttile(tabs);
    hold(c, 'on');
    stages = ["exposure", "solve", "settle"];
    colors = [0.14 0.45 0.69; 0.84 0.39 0.14; 0.15 0.53 0.43];
    for stage = 1:numel(stages)
        rows = find(timeline.stage == stages(stage));
        for i = rows'
            plot(c, [timeline.start_s(i), timeline.end_s(i)], [stage, stage], ...
                'LineWidth', 12, 'Color', colors(stage,:));
        end
    end
    yticks(c, 1:3);
    yticklabels(c, {'Exposure','Plate solve','Settle'});
    ylim(c, [0.5 3.5]);
    xlim(c, [0, timeline.end_s(end) + 1]);
    grid(c, 'on');
    title(c, sprintf('Capture sequence   |   %d exposures, modeled solve time 8 s each', p.frames));
    xlabel(c, 'Time from first exposure (s)');
    title(tabs, sprintf('StarTracker Pi   |   Kp %.3f   Ki %.3f   Kd %.3f', p.kp, p.ki, p.kd));
    exportgraphics(fig, file, 'Resolution', 150);
end
