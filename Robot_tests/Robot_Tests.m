%% Variables Initialization

angleX = 0;
angleY = 0;
angleZ = 0;

Accel_pitch = 0;
acc_roll = 0;
pitch=[];
pitch2 = [];
roll = [];

Gyro_data_x = [];

angleXarray = zeros(length(pitch40pausado.GyrX), 1);
angleYarray = zeros(length(pitch40pausado.GyrY), 1);
angleZarray = zeros(length(pitch40pausado.GyrZ), 1);

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

%% Process Robot data
roll120_downsample = resample(str2double(roll120.Robot), 500, 100);

roll120_downsample = rad2deg(roll120_downsample);

%% Gyroscope angle estimation

for i = 2:length(Gyro_data_x)
    angleX = angleX + Gyro_data_x(i) * (1/fs);%%((Gyro_data_x(i-1) + Gyro_data_x(i))/(2*fs));
    angleY = angleY + Gyro_data_y(i) * (1/fs);%%((Gyro_data_y(i-1) + Gyro_data_y(i))/(2*fs));
    angleZ = angleZ + Gyro_data_z(i) * (1/fs);%%((Gyro_data_z(i-1) + Gyro_data_z(i))/(2*fs));

    angleXarray(i) = angleX;
    angleYarray(i) = angleY;
    angleZarray(i) = angleZ;
end

%% Accelerometer angle estimation
Acc_data_x = Acc_data_x/max(Acc_data_x);
Acc_data_y = Acc_data_y/max(Acc_data_y);
Acc_data_z = Acc_data_z/max(Acc_data_z);

pitch = [];
pitch2 = [];
for i=1:length(Acc_data_x)
    %Accel_pitch = asin(accx(i)/(sqrt(accx(i)^2 + accy(i)^2 + accz(i)^2)));
    %Accel_roll = atan2(accx(i),accy(i));

    if Acc_data_x(i) < 0 && abs(Acc_data_z(i)) < eps
        acc_pitch = 180;  % or -180, choose the appropriate value
    else
        acc_pitch = wrapTo180(atan2d(Acc_data_z(i), Acc_data_x(i)));
    end

    %acc_pitch2 = rad2deg(atan2(Acc_data_y(i),Acc_data_x(i)));

    %acc_roll = rad2deg(atan2(accx(i),accz(i)));

    pitch = [pitch; acc_pitch];
    %pitch2 = [pitch2; acc_pitch2];
    %roll = [roll acc_roll];
end

%pitch_norm = pitch/180;
%pitch2_norm = pitch2/180;


