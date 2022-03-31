clc
clear all
close all

%% initialization
x = 0.0;
y = 0.1;
l1 = 0.2;
l2 = 0.2;

%% case_1
a = x^2 + l1^2 + y^2 - l2^2;
b = 2*x*l1;
c = 2*y*l1;

theta1_case1_solution1 = asin(((2*a*b + sqrt(4*a^2*b^2 - 4*(b^2+c^2)*(a^2-c^2)))/(2*(b^2+c^2))));
theta1_case1_solution2 = asin(((2*a*b - sqrt(4*a^2*b^2 - 4*(b^2+c^2)*(a^2-c^2)))/(2*(b^2+c^2))));
theta2_case1_solution1 = asin((x - l1*sin(theta1_case1_solution1))/l2);
theta2_case1_solution2 = asin((x - l1*sin(theta1_case1_solution2))/l2);

fprintf('#######################\n');
fprintf('theta1_case1_solution1 is %f\n',theta1_case1_solution1*180/3.14);
fprintf('theta2_case1_solution1 is %f\n',theta2_case1_solution1*180/3.14);
fprintf('\n');
fprintf('theta1_case1_solution2 is %f\n',theta1_case1_solution2*180/3.14);
fprintf('theta2_case1_solution2 is %f\n',theta2_case1_solution2*180/3.14);
fprintf('#######################\n');
fprintf('\n');
fprintf('\n');
fprintf('\n');

%% case_2
a = x^2 + l1^2 + y^2 - l2^2;
b = 2*x*l1;
c = -2*y*l1;

theta1_case2_solution1 = asin(((2*a*b + sqrt(4*a^2*b^2 - 4*(b^2+c^2)*(a^2-c^2)))/(2*(b^2+c^2))));
theta1_case2_solution2 = asin(((2*a*b - sqrt(4*a^2*b^2 - 4*(b^2+c^2)*(a^2-c^2)))/(2*(b^2+c^2))));
theta2_case2_solution1 = asin((x - l1*sin(theta1_case2_solution1))/l2);
theta2_case2_solution2 = asin((x - l1*sin(theta1_case2_solution2))/l2);

fprintf('#######################\n');
fprintf('theta1_case2_solution1 is %f\n',theta1_case2_solution1*180/3.14);
fprintf('theta2_case2_solution1 is %f\n',theta2_case2_solution1*180/3.14);
fprintf('\n');
fprintf('theta1_case2_solution2 is %f\n',theta1_case2_solution2*180/3.14);
fprintf('theta2_case2_solution2 is %f\n',theta2_case2_solution2*180/3.14);
fprintf('#######################\n');