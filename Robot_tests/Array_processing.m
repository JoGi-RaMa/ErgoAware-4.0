%% Array alignment

imu_data = pitch_comp_final;
robot_data = robot_data_final;

[cycle1_imu, cycle1_robot] = alignsignals(imu_data(1010:3682), robot_data(910:4081));
[cycle2_imu, cycle2_robot] = alignsignals(imu_data(3683:6347), robot_data(4082:7241));
[cycle3_imu, cycle3_robot] = alignsignals(imu_data(6348:9011), robot_data(7242:10391));
[cycle4_imu, cycle4_robot] = alignsignals(imu_data(9012:11680), robot_data(10392:13542));
[cycle5_imu, cycle5_robot] = alignsignals(imu_data(11681:end), robot_data(13543:end));


%% Cycle 1 Array processing

cycle1_robot_processed = cycle1_robot;
cycle1_imu_processed = cycle1_imu;

if length(cycle1_imu) > length(cycle1_robot)
    for i=length(cycle1_robot):length(cycle1_imu)
        cycle1_robot_processed(i) = cycle1_robot(length(cycle1_robot));
    end

elseif length(cycle1_robot) > length(cycle1_imu)
    for i=length(cycle1_imu):length(cycle1_robot)
        cycle1_imu_processed(i) = cycle1_imu(length(cycle1_imu));
    end

end

%% Cycle 2 Array processing
cycle2_robot_processed = cycle2_robot;
cycle2_imu_processed = cycle2_imu;

if length(cycle2_imu) > length(cycle2_robot)
    for i=length(cycle2_robot):length(cycle2_imu)-1
        cycle2_robot_processed = [cycle2_robot_processed; cycle2_robot(length(cycle2_robot))];
    end

elseif length(cycle2_robot) > length(cycle2_imu)
    for i=length(cycle2_imu):length(cycle2_robot)-1
        cycle2_imu_processed = [cycle2_imu_processed; cycle2_imu(length(cycle2_imu))];
    end

end

%% Cycle 3 Array processing
cycle3_robot_processed = cycle3_robot;
cycle3_imu_processed = cycle3_imu;

if length(cycle3_imu) > length(cycle3_robot)
    for i=length(cycle3_robot):length(cycle3_imu)-1
        cycle3_robot_processed = [cycle3_robot_processed; cycle3_robot(length(cycle3_robot))];
    end

elseif length(cycle3_robot) > length(cycle3_imu)
    for i=length(cycle3_imu):length(cycle3_robot)-1
        cycle3_imu_processed = [cycle3_imu_processed; cycle3_imu(length(cycle3_imu))];
    end

end

%% Cycle 4 Array processing
cycle4_robot_processed = cycle4_robot;
cycle4_imu_processed = cycle4_imu;

if length(cycle4_imu) > length(cycle4_robot)
    for i=length(cycle4_robot):length(cycle4_imu)-1
        cycle4_robot_processed = [cycle4_robot_processed; cycle4_robot(length(cycle4_robot))];
    end

elseif length(cycle4_robot) > length(cycle4_imu)
    for i=length(cycle4_imu):length(cycle4_robot)-1
        cycle4_imu_processed = [cycle4_imu_processed; cycle4_imu(length(cycle4_imu))];
    end

end

%% Cycle 5 Array processing
cycle5_robot_processed = cycle5_robot;
cycle5_imu_processed = cycle5_imu;

if length(cycle5_imu) > length(cycle5_robot)
    for i=length(cycle5_robot):length(cycle5_imu)-1
        cycle5_robot_processed = [cycle5_robot_processed; cycle5_robot(length(cycle5_robot))];
    end

elseif length(cycle5_robot) > length(cycle5_imu)
    for i=length(cycle5_imu):length(cycle5_robot)-1
        cycle5_imu_processed = [cycle5_imu_processed; cycle5_imu(length(cycle5_imu))];
    end

end

%% Array concatenation
imu_data_align = [];
robot_data_align = [];

imu_data_align = [imu_data_align; cycle1_imu_processed; cycle2_imu_processed; cycle3_imu_processed; cycle4_imu_processed; cycle5_imu_processed];
robot_data_align = [robot_data_align; cycle1_robot_processed; cycle2_robot_processed; cycle3_robot_processed; cycle4_robot_processed; cycle5_robot_processed];

%% RMS Error
residuals = robot_data_align - imu_data_align;
squared_residuals = residuals.^2;
mean_squared_residuals = mean(squared_residuals);

rms_error = sqrt(mean_squared_residuals);