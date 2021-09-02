% 空间刚体变换。
% 输入：pcIn要求为无组织点云，modelNormal为平面的法向量
% 输出：pcOut要求为无组织点云
function [pcOut] = pcTransform(pcIn, model) 

modelNormal = model.Normal;
X_normal = [1 0 0];
Y_normal = [0 1 0];
% Z_normal = [0 0 1];
alpha = acos(dot(modelNormal, X_normal) / (norm(modelNormal) * norm(X_normal)));
beta = acos(dot(modelNormal, Y_normal) / (norm(modelNormal) * norm(Y_normal)));
theta = 0;

alpha = -(alpha - pi / 2);
beta = (beta - pi / 2);

% 旋转矩阵
R = [cos(beta) * cos(theta)                                         cos(beta) * sin(theta)                                          -sin(beta);
    -cos(alpha) * sin(theta) + sin(alpha) * sin(beta) * cos(theta)  cos(alpha) * cos(theta) + sin(alpha) * sin(beta) * sin(theta)   sin(alpha) * cos(beta);
    sin(alpha) * sin(theta) + cos(alpha) * sin(beta) * cos(theta)   -sin(alpha) * cos(theta) + cos(alpha) * sin(beta) * sin(theta)  cos(alpha) * cos(beta)];

Temp(:, 1) = pcIn.Location(:, 1);
Temp(:, 2) = pcIn.Location(:, 2);
Temp(:, 3) = pcIn.Location(:, 3);

% 旋转
Temp = Temp * R;
% Temp(:, 3) = Temp(:, 3) - min(Temp(:, 3));

% 平移
a = model.Parameters(1);
b = model.Parameters(2);
c = model.Parameters(3);
d = model.Parameters(4);

X = -150 : 0.1 : 150;
Y = -150 : 0.1 : 150;
panelZ = -(a * X + b * Y + d) / c;

Temp(:, 3) = Temp(:, 3) - median(panelZ);

pcOut = pointCloud(Temp);

% figure;
% pcshow(pcIn);
% 
% figure;
% pcshow(pcOut);
