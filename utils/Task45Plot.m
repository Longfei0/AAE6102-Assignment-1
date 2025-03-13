% Script for visualizing GNSS processing results
% Includes multi-correlator outputs, WLS and Kalman filter position/velocity results

%% Check if required data exists
if ~exist('trackResults', 'var') || ~exist('navSolutions', 'var')
    warning('trackResults or navSolutions data missing. Please run GNSS processor first');
    return;
end

%% Multi-correlator output plotting
figure('Name', 'Multi-Correlator Output');

try
    data = trackResults(4).I_multi{201};

    if data(6) < 0
        data = -data;
    end

    plot(-0.5:0.1:0.5, data, 'r-'); hold on;
    scatter(-0.5:0.1:0.5, data, 'bo');
    xlabel('Code Phase Delay (chips)');
    ylabel('Auto-Correlation Function (ACF) Value');
    title('Multi-Correlator Auto-Correlation Function Output');
    grid on;
catch
    warning('Multi-correlator data missing or invalid format');
end

%% WLS Positioning Results Plotting
try

    if isfield(navSolutions, 'latitude') && ~isempty(navSolutions.latitude)
        figure('Name', 'WLS Positioning Results');
        city_gt = [22.3198722, 114.209101777778, 3]; % Ground truth reference point

        % Create geographic axes
        geoaxes;
        geobasemap satellite;

        % Plot estimated positions
        for i = 1:size(navSolutions.latitude, 2)
            geoplot(navSolutions.latitude(i), navSolutions.longitude(i), 'r*', 'MarkerSize', 10);
            hold on;
        end

        % Plot ground truth
        geoplot(city_gt(1), city_gt(2), 'o', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'MarkerEdgeColor', 'y');
        title('WLS Positioning Results (Red*: Estimates, Yellowo: Ground Truth)');
    end

catch
    warning('WLS positioning data missing or invalid format');
end

%% WLS Velocity Estimation Plotting
try

    if isfield(navSolutions, 'vX') && ~isempty(navSolutions.vX)
        figure('Name', 'WLS Velocity Estimation');
        v = [];

        for i = 1:size(navSolutions.vX, 2)
            v = [v; navSolutions.vX(i), navSolutions.vY(i), navSolutions.vZ(i)];
        end

        subplot(2, 1, 1);
        plot(1:size(v, 1), v(:, 1), '-b', 1:size(v, 1), v(:, 2), '-r');
        legend('X Velocity (ECEF)', 'Y Velocity (ECEF)');
        xlabel('Epoch Number');
        ylabel('Velocity (m/s)');
        title('WLS Estimated X/Y Velocities');
        grid on;

        subplot(2, 1, 2);
        plot(1:size(v, 1), v(:, 3), '-g');
        legend('Z Velocity (ECEF)');
        xlabel('Epoch Number');
        ylabel('Velocity (m/s)');
        title('WLS Estimated Z Velocity');
        grid on;
    else
        warning('WLS velocity data unavailable for plotting');
    end

catch
    warning('WLS velocity data missing or invalid format');
end

%% Kalman Filter Positioning Results
try

    if isfield(navSolutions, 'latitude_kf') && ~isempty(navSolutions.latitude_kf)
        figure('Name', 'Kalman Filter Positioning Results');
        city_gt = [22.3198722, 114.209101777778, 3]; % Ground truth reference point

        % Create geographic axes
        geoaxes;
        geobasemap satellite;

        % Plot estimated positions
        for i = 1:size(navSolutions.latitude_kf, 2)
            geoplot(navSolutions.latitude_kf(i), navSolutions.longitude_kf(i), 'r*', 'MarkerSize', 10);
            hold on;
        end

        % Plot ground truth
        geoplot(city_gt(1), city_gt(2), 'o', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'MarkerEdgeColor', 'y');
        title('Kalman Filter Positioning Results (Red*: Estimates, Yellowo: Ground Truth)');
    end

catch
    warning('Kalman filter positioning data missing or invalid format');
end

%% Kalman Filter Velocity Estimation Plotting
try

    if isfield(navSolutions, 'VX_kf') && ~isempty(navSolutions.VX_kf)
        figure('Name', 'Kalman Filter Velocity Estimation');
        v = [];

        for i = 1:size(navSolutions.VX_kf, 2)
            v = [v; navSolutions.VX_kf(i), navSolutions.VY_kf(i), navSolutions.VZ_kf(i)];
        end

        subplot(2, 1, 1);
        plot(1:size(v, 1), v(:, 1), '-b', 1:size(v, 1), v(:, 2), '-r');
        legend('X Velocity (ECEF)', 'Y Velocity (ECEF)');
        xlabel('Epoch Number');
        ylabel('Velocity (m/s)');
        title('Kalman Filter Estimated X/Y Velocities');
        grid on;

        subplot(2, 1, 2);
        plot(1:size(v, 1), v(:, 3), '-g');
        legend('Z Velocity (ECEF)');
        xlabel('Epoch Number');
        ylabel('Velocity (m/s)');
        title('Kalman Filter Estimated Z Velocity');
        grid on;
    else
        warning('Kalman filter velocity data unavailable for plotting');
    end

catch
    warning('Kalman filter velocity data missing or invalid format');
end
