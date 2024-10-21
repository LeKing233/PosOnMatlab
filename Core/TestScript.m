% TestScript.m
%
% This script tests the quaternion library functions to ensure that each
% function output is consistent.
%
% Date          Author          Notes
% 27/09/2011    SOH Madgwick    Initial release

%% Start of script

close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% ZYX Euler angles to rotation matrix
clc;                                % clear the command terminal
clear;                              % clear all variables

Phi = [89, 30, 0];
R = Utils.getCbnFromPhi(Phi);
num = ' % 1.5f';
a = sprintf('\rAxis-angle to rotation matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), R(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), R(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), R(3,:));
disp(strcat(a,b,c,d));

%% ZYX Euler angles to quaternion
clc;                                % clear the command terminal
clear;                              % clear all variables

Phi = [-8.32, 157.2, -20.7];
q = Utils.getQFromPhi(Phi);
num = ' % 1.5f';
a = sprintf('\rAxis-angle to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

%% Quaternion to rotation matrix
clc;                                % clear the command terminal
clear;                              % clear all variables

q = [ 0.18115	  0.16155	  0.96436	 -0.10537]';
q = quatnormalize(q')';
R = Utils.getCbnFromQ(q);
num = ' % 1.5f';
a = sprintf('\rQuaternion to rotation matrix:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), R(1,:));
c = sprintf(strcat('\r', num, '\t', num, '\t', num), R(2,:));
d = sprintf(strcat('\r', num, '\t', num, '\t', num), R(3,:));
disp(strcat(a,b,c,d));

%% Rotation matrix to quaternion
clc;                                % clear the command terminal
clear;                              % clear all variables

R = [-0.88217	  0.34976	  0.31534
  0.27341	  0.92560	 -0.26176
 -0.38343	 -0.14470	 -0.91216];

q = Utils.getQFromCbn(R);
% q = rotm2quat(R);
q = quatnormalize(q')';
num = ' % 1.5f';
a = sprintf('\rRotation matrix to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q);
disp(strcat(a,b));

%% Rotation matrix to ZYX Euler angles
clc;                                % clear the command terminal
clear;                              % clear all variables

R = [ -0.6427876,  0.0267346, -0.7655778;
   0.7660444,  0.0224330, -0.6423960;
  -0.0000000, -0.9993908, -0.0348995];

euler = Utils.getPhiFromCbn(R);
num = ' % 1.5f';
a = sprintf('\rRotation matrix to ZYX Euler angles:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), euler);
disp(strcat(a,b));

%% Quaternion to ZYX Euler angles
clc;                                % clear the command terminal
clear;                              % clear all variables

q = [0.18115	  0.16155	  0.96436	 -0.10537]';
euler = Utils.getPhiFromQ(q);
num = ' % 1.5f';
a = sprintf('\rQuaternion to ZYX Euler angles:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num), euler);
disp(strcat(a,b));

%%
clc
clear
q = [0.3107;
   -0.0176;
   -0.0509;
    0.9490];
q1 = Utils.normalizeQ(q);
num = ' % 1.5f';
a = sprintf('\rAxis-angle to quaternion:');
b = sprintf(strcat('\r', num, '\t', num, '\t', num, '\t', num), q1);
disp(strcat(a,b));
%% End of script