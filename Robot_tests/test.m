angleX = 0;
angleY = 0;
angleZ = 0;

angleXarray = zeros(length(pitch1.GyrX), 1);
angleYarray = zeros(length(pitch1.GyrY), 1);
angleZarray = zeros(length(pitch1.GyrZ), 1);

fs = 900;

for i = 2:length(pitch1.GyrX)
    angleX = angleX + ((pitch1.GyrX(i-1) + pitch1.GyrX(i))/(2*fs));
    angleY = angleY + ((pitch1.GyrY(i-1) + pitch1.GyrY(i))/(2*fs));
    angleZ = angleZ + ((pitch1.GyrZ(i-1) + pitch1.GyrZ(i))/(2*fs));

    angleXarray(i) = angleX;
    angleYarray(i) = angleY;
    angleZarray(i) = angleZ;
end