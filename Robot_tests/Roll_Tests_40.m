%% Variables Initialization

angleX = 0;
angleY = 0;
angleZ = 0;

Accel_roll = 0;

fs = 535;

%% Process IMU data

Gyro_data_x = [];
Gyro_data_y = [];
Gyro_data_z = [];

Acc_data_x = [];
Acc_data_y = [];
Acc_data_z = [];

Gyro_data_index = roll40.GyrX ~= 0, 1:3; %índices

for i = 1:length(Gyro_data_index)
    if(Gyro_data_index(i) == 1)
        Gyro_data_x = [Gyro_data_x; roll40.GyrX(i)];
        Gyro_data_y = [Gyro_data_y; roll40.GyrY(i)];
        Gyro_data_z = [Gyro_data_z; roll40.GyrZ(i)];
    
    else
        Acc_data_x = [Acc_data_x; roll40.AccX(i)];
        Acc_data_y = [Acc_data_y; roll40.AccY(i)];
        Acc_data_z = [Acc_data_z; roll40.AccZ(i)];

    end 
end

Acc_data_x = resample(Acc_data_x, 80, 100);
Acc_data_y = resample(Acc_data_y, 80, 100);
Acc_data_z = resample(Acc_data_z, 80, 100);

%% Process Robot data

robot_data = resample(roll40robot.Angle, 80, 484);

robot_data = -rad2deg(robot_data);

%% Accelerometer angle estimation
Acc_data_x = Acc_data_x/max(Acc_data_x);
Acc_data_y = Acc_data_y/max(Acc_data_y);
Acc_data_z = Acc_data_z/max(Acc_data_z);

acc_roll = 0;

roll = [];

label = [];

for i=1:length(Acc_data_x)

    raw_angle = atan2d(Acc_data_y(i), -Acc_data_x(i));

    if(Acc_data_x(i) > 0.7 && Acc_data_y(i) < 0.05 && Acc_data_y(i) > -0.05)
        if(roll(i-1) > 170)
            acc_roll = 180;
        elseif(roll(i-1) < -170)
            acc_roll = -180;
        end
    else
        acc_roll = raw_angle;
    end

    roll = [roll; acc_roll];
end

 roll = 180 + roll;

roll_norm =  roll/180;

roll_index = find(roll < 170, 1, "first");

for i=1:roll_index
    roll(i) = 0;
end

%% Gyroscope angle estimation

% angleXarray = zeros(length(Gyro_data_x), 1);
% angleYarray = zeros(length(Gyro_data_y), 1);
% angleZarray = zeros(length(Gyro_data_z), 1);

angleXarray = [];
angleYarray = [];
angleZarray = [];

alpha = 0.982207;
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

    roll_comp = (alpha * roll(i)) + ((1 - alpha) * angleZ);

    anglexarray = [angleXarray; angleX];
    angleYarray = [angleYarray; angleY];
    angleZarray = [angleZarray; roll_comp];
end

%% Array processing

robot_data_final = -robot_data;
roll_comp_final = angleZarray;

if length(angleZarray) > length(robot_data)
    for i=length(robot_data):length(angleZarray)-1
        robot_data_final = [robot_data_final; robot_data(length(robot_data))];
    end

elseif length(robot_data) > length(angleZarray)
    for i=length(angleZarray):length(robot_data)-1
        roll_comp_final = [roll_comp_final; angleZarray(length(angleZarray))];
    end

end

%% Dataset alignment

index_x_start = find(robot_data_final > 0.01, 1, "first");
index_y_start = find(roll_comp_final > 0.01, 1, "first");

align_robot_data = robot_data_final(index_x_start:end, :);
align_imu_data = angleZarray(index_y_start:end, :);

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

array_imu = array_imu_aligned(1:6529);
array_robot = array_robot_aligned(1:6529);

%% Error calculation
rms_error = [];

residuals = array_imu - array_robot;
squared_residuals = residuals.^2;
mean_squared_residuals = mean(squared_residuals);

rms_error = sqrt(mean_squared_residuals);