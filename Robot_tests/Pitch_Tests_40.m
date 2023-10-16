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

Gyro_data_index = pitch40.GyrX ~= 0, 1:3; %índices

for i = 1:length(Gyro_data_index)
    if(Gyro_data_index(i) == 1)
        Gyro_data_x = [Gyro_data_x; pitch40.GyrX(i)];
        Gyro_data_y = [Gyro_data_y; pitch40.GyrY(i)];
        Gyro_data_z = [Gyro_data_z; pitch40.GyrZ(i)];
    
    else
        Acc_data_x = [Acc_data_x; pitch40.AccX(i)];
        Acc_data_y = [Acc_data_y; pitch40.AccY(i)];
        Acc_data_z = [Acc_data_z; pitch40.AccZ(i)];

    end 
end

Acc_data_x = resample(Acc_data_x, 80, 100);
Acc_data_y = resample(Acc_data_y, 80, 100);
Acc_data_z = resample(Acc_data_z, 80, 100);



%% Process Robot data

robot_data = resample(pitch40robot.Angle, 80, 473);

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

    if(Acc_data_x(i) > 0.7 && Acc_data_z(i) < 0.05 && Acc_data_z(i) > -0.05)
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

pitch_index = find(pitch == 0, 1, "first");

for i=1:pitch_index
    pitch(i) = 0;
end

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
    for i=length(robot_data):length(angleYarray)
        robot_data_final(i) = robot_data(length(robot_data));
    end

elseif length(robot_data) > length(angleYarray)
    for i=length(angleYarray):length(robot_data)
        pitch_comp_final(i) = angleYarray(length(angleYarray));
    end

end

%% Dataset alignment

index_x_start = find(robot_data_final > 0.01, 1, "first");
index_y_start = 110 + find(pitch_comp_final(110:end) > 0.01, 1, "first");

align_robot_data = robot_data(index_x_start:end, :);
align_imu_data = pitch(index_y_start:end, :);

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

%% Signal Filtering 

order = 4;
low_freq = 0.1;

[b, a] = butter(order, low_freq, "low");

array_imu = filter(b, a, array_imu);

%% Align Signals

[array_imu_aligned, array_robot_aligned] = alignsignals(array_imu, array_robot, Method = "risetime");

m = zeros((length(array_imu_aligned)-length(array_robot_aligned)), 1);

array_robot_aligned = [array_robot_aligned; m];

%% Last cycle cutoff

array_imu = array_imu_aligned(1:6539);
array_robot = array_robot_aligned(1:6539);


%% Error calculation
rms_error = [];

residuals = array_robot - array_imu;
squared_residuals = residuals.^2;
mean_squared_residuals = mean(squared_residuals);

rms_error = sqrt(mean_squared_residuals);
