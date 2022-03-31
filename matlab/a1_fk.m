clc
clear all
close all
%{
generate forward equation for FR leg:
    --- x0 y0 z0 that is trunk frame
    --- x1 y1 z1 that is hip frame
    --- x2 y2 z2 that is thigh frame
    --- x3 y3 z3 that is calf frame
    --- mujoco all kids frame have fixed perspective (unit euler matrix)
    --- theta_hip, theta_thigh, theta_calf
%}

% homogeneous between world and trunk first we assume trunk not moving thus x-roll y-pitch z-yaw is fixed thus rotation matrix is unit
H01 = [1, 0, 0, 0;
       0, 1, 0, 0;
       0, 0, 1, 0.6;
       0, 0, 0, 1];
   
% homogeneous between trunk and hip R_roll
syms theta_hip
R12 = [1, 0,               0;
       0, cos(theta_hip), -sin(theta_hip);
       0, sin(theta_hip),  cos(theta_hip)];
D12 = [0.183;-0.047;0];
H12(1:3,1:3) = R12;
H12(1:3,4) = D12;
H12(4,1:4) = [0,0,0,1];

% homogeneous between hip and thigh R_pitch
syms theta_thigh
R23 = [cos(theta_thigh), 0, sin(theta_thigh);
       0,                1,                0;
      -sin(theta_thigh), 0, cos(theta_thigh)];
D23 = [0;-0.08505;0];
H23(1:3,1:3) = R23;
H23(1:3,4) = D23;
H23(4,1:4) = [0,0,0,1];

% homogeneous between thigh and calf R_pitch
syms theta_calf
R34 = [cos(theta_calf), 0, sin(theta_calf);
       0,               1,               0;
      -sin(theta_calf), 0, cos(theta_calf)];
D34 = [0;0;-0.2];
H34(1:3,1:3) = R34;
H34(1:3,4) = D34;
H34(4,1:4) = [0,0,0,1];

% find the foot position
p_foot = H01*H12*H23*H34*[0;0;-0.2;1];
p_foot = p_foot(1:3,1);
p_foot = vpa(simplify(p_foot));

% jacobian generation --- in this case, we have 3 unknow variables and 3 functions so that Newton iteration root finding method can be used
F = [0.183 - 0.2*sin(theta_thigh) - 0.2*sin(theta_calf + theta_thigh),
    0.2*cos(theta_thigh)*sin(theta_hip) - 0.08505*cos(theta_hip) - 0.2*sin(theta_hip)*sin(theta_calf)*sin(theta_thigh) + 0.2*cos(theta_calf)*cos(theta_thigh)*sin(theta_hip) - 0.047,
    0.2*cos(theta_hip)*sin(theta_calf)*sin(theta_thigh) - 0.2*cos(theta_hip)*cos(theta_thigh) - 0.08505*sin(theta_hip) - 0.2*cos(theta_hip)*cos(theta_calf)*cos(theta_thigh) + 0.6];
q = [theta_hip,theta_thigh,theta_calf];
J = vpa(simplify(jacobian(F,q)));
Jinv = vpa(simplify(inv(J)))









