%% Variables Initialization

angleX = 0;
angleY = 0;
angleZ = 0;

Accel_pitch = 0;
acc_roll = 0;
pitch = [];

fs = 535;

%% Process IMU data

Gyro_data_x = [];
Gyro_data_y = [];
Gyro_data_z = [];

Acc_data_x = [];
Acc_data_y = [];
Acc_data_z = [];

Gyro_data_index = pitch120.GyrX ~= 0, 1:3; %índices

for i = 1:length(Gyro_data_index)
    if(Gyro_data_index(i) == 1)
        Gyro_data_x = [Gyro_data_x; pitch120.GyrX(i)];
        Gyro_data_y = [Gyro_data_y; pitch120.GyrY(i)];
        Gyro_data_z = [Gyro_data_z; pitch120.GyrZ(i)];
    
    else
        Acc_data_x = [Acc_data_x; pitch120.AccX(i)];
        Acc_data_y = [Acc_data_y; pitch120.AccY(i)];
        Acc_data_z = [Acc_data_z; pitch120.AccZ(i)];

    end 
end

Acc_data_x = resample(Acc_data_x, 80, 100);
Acc_data_y = resample(Acc_data_y, 80, 100);
Acc_data_z = resample(Acc_data_z, 80, 100);

%% Process Robot data

robot_data = resample(pitch120robot.Angle, 80, 482);

robot_data = -rad2deg(robot_data);

%% Accelerometer angle estimation
Acc_data_x = Acc_data_x/max(Acc_data_x);
Acc_data_y = Acc_data_y/max(Acc_data_y);
Acc_data_z = Acc_data_z/max(Acc_data_z);

acc_pitch = 0;

pitch = [];

label = [];

for i=1:length(Acc_data_x)

    raw_angle = atan2d(Acc_data_z(i), -Acc_data_x(i));

    if(Acc_data_x(i) > 0.6 && Acc_data_z(i) < 0.05 && Acc_data_z(i) > -0.05)
        if(pitch(i-1) > 170)
            acc_pitch = 180;
        elseif(pitch(i-1) < -170)
            acc_pitch = -180;
        end
    else
        acc_pitch = raw_angle;
    end

    pitch = [pitch; acc_pitch];
end

pitch = 180 - pitch;

%pitch_norm =  pitch/180;

pitch_index = find(pitch < 170, 1, "first");

for i=1:pitch_index
    pitch(i) = 0;
end

%% Filtering IMU data

filter_order = 4;
low_freq = 0.1;

[b, a] = butter(filter_order,low_freq, "low");

pitch = filter(b, a, pitch);

%% Gyroscope angle estimation

% angleXarray = zeros(length(Gyro_data_x), 1);
% angleYarray = zeros(length(Gyro_data_y), 1);
% angleZarray = zeros(length(Gyro_data_z), 1);

angleXarray = [];
angleYarray = [];
angleZarray = [];

alpha = 0.981646;
num_groups = length(Gyro_data_x)/7.5;

for i = 1:num_groups

    gyro6_samples_X = Gyro_data_x((i-1)*7 + 1 : i*7);
    gyro6_samples_Y = Gyro_data_y((i-1)*7 + 1 : i*7);
    gyro6_samples_Z = Gyro_data_z((i-1)*7 + 1 : i*7);

    for j=2:length(gyro6_samples_X)
        angleX = angleX + ((gyro6_samples_X(j-1) + gyro6_samples_X(j))/(2*fs));
        angleY = angleY + ((gyro6_samples_Y(j-1) + gyro6_samples_Y(j))/(2*fs));
        angleZ = angleZ + ((gyro6_samples_Z(j-1) + gyro6_samples_Z(j))/(2*fs));
    end

    pitch_comp = (alpha * pitch(i)) + ((1 - alpha) * angleY);

    anglexarray = [angleXarray; angleX];
    angleYarray = [angleYarray; pitch_comp];
    angleZarray = [angleZarray; angleZ];
end

%% Array processing

robot_data_final = robot_data;
pitch_comp_final = angleYarray;

if length(angleYarray) > length(robot_data)
    for i=length(robot_data):length(angleYarray)-1
        robot_data_final = [robot_data_final; robot_data(length(robot_data))];
    end

elseif length(robot_data) > length(angleYarray)
    for i=length(angleYarray):length(robot_data)-1
        pitch_comp_final = [pitch_comp_final; angleYarray(length(angleYarray))];
    end

end


%% Dataset alignment

index_x_start = find(robot_data_final > 0.01, 1, "first");
index_y_start = find(pitch_comp_final > 0.01, 1, "first");

align_robot_data = robot_data(index_x_start:end, :);
align_imu_data = angleYarray(index_y_start:end, :);

align_robot_data_final = align_robot_data;
align_imu_data_final = align_imu_data;

if length(align_robot_data) > length(align_imu_data)
    for i=length(align_imu_data):length(align_robot_data)-1
        align_imu_data_final = [align_imu_data_final; align_imu_data(length(align_imu_data))];
    end

elseif length(align_imu_data) > length(align_robot_data)
    for i=length(align_robot_data):length(align_imu_data)-1
        align_robot_data_final = [align_robot_data_final; align_robot_data(length(align_robot_data))];
    end

end

array_imu = align_imu_data_final;
array_robot = align_robot_data_final;

%% Align Signals

[array_imu_aligned, array_robot_aligned] = alignsignals(array_imu, array_robot, Method = "risetime");

m = zeros((length(array_imu_aligned)-length(array_robot_aligned)), 1);

array_robot_aligned = [array_robot_aligned; m];

%% Last cycle cutoff

array_imu = array_imu_aligned(1:2240);
array_robot = array_robot_aligned(1:2240);

%% Error calculation
rms_error = [];

residuals = align_robot_data_final - align_imu_data_final;
squared_residuals = residuals.^2;
mean_squared_residuals = mean(squared_residuals);

rms_error = sqrt(mean_squared_residuals);