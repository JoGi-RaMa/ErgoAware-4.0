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

Gyro_data_index = pitch40pausado.GyrX ~= 0, 1:3; %índices

for i = 1:length(Gyro_data_index)
    if(Gyro_data_index(i) == 1)
        Gyro_data_x = [Gyro_data_x; pitch40pausado.GyrX(i)];
        Gyro_data_y = [Gyro_data_y; pitch40pausado.GyrY(i)];
        Gyro_data_z = [Gyro_data_z; pitch40pausado.GyrZ(i)];
    
    else
        Acc_data_x = [Acc_data_x; pitch40pausado.AccX(i)];
        Acc_data_y = [Acc_data_y; pitch40pausado.AccY(i)];
        Acc_data_z = [Acc_data_z; pitch40pausado.AccZ(i)];

    end 
end

angleYarray = resample(angleYarray, 100, 600);

%% Process Robot data
roll120_downsample = resample(str2double(roll120.Robot), 500, 100);

roll120_downsample = rad2deg(roll120_downsample);

%% Gyroscope angle estimation

angleXarray = zeros(length(Gyro_data_x), 1);
angleYarray = zeros(length(Gyro_data_y), 1);
angleZarray = zeros(length(Gyro_data_z), 1);

for i = 2:length(Gyro_data_x)
    angleX = angleX + (Gyro_data_x(i) * (1/fs));%%((Gyro_data_x(i-1) + Gyro_data_x(i))/(2*fs));
    angleY = angleY + (Gyro_data_y(i) * (1/fs));%%((Gyro_data_y(i-1) + Gyro_data_y(i))/(2*fs));
    angleZ = angleZ + (Gyro_data_z(i) * (1/fs));%%((Gyro_data_z(i-1) + Gyro_data_z(i))/(2*fs));

    angleXarray(i) = angleX;
    angleYarray(i) = angleY;
    angleZarray(i) = angleZ;
end

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

%% Complementary Filter
alpha = 0.981646;

pitch_comp = (alpha * pitch) + ((1 - alpha) * angleYarray);


