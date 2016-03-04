% PURPOSE:  Intialize workspace for crazyflie position control in simulink
% FILENAME: PosCntrl_Params.m
% AUTHOR:   Roberto Shu
% LAST EDIT: 3/4/2016
%------------------- Instructions ---------------------------

% ------------------------------------------
%   Initialize workspace
% -----------------------------------------
% Clear workspace
clear all; close all; clc

% Add to path
addpath('Optitrack')
addpath('RealTime_Pacer')
addpath('CrazyFlie_lib')

% ------------------------------------------
%   System Parameters
% -----------------------------------------

% Enviroment 
g = 9.81;               % Acceleration due to gravity [m/s]

% Crazyflie
m = 0.037;                 % mass w/ markers+frame [kg]
omega_h = 37500*1.15;      % 1.15 PWM for hover
omega_h = 17500;
k_f = m*g/omega_h^2;        % Proportional gain relating thrust [N] and angular velocity [PWM]

% Control
sample_frequency = 100;
sample_time = 1/sample_frequency;

% ------------------------------------------
%   Position Control Gains
% ------------------------------------------
% Y-direcction acc
Kpy = 1.2;
Kdy = 0.4;

% X - direction acc
Kpx = 1.2;
Kdx = 0.4;

% Z - direction 
Kpz = 1.2;
Kdz = 0.4;

% ------------------------------------------
%   Angle/Thrust limits
% ------------------------------------------
roll_lb = -40*pi/180;
roll_ub = 40*pi/180;

pitch_lb = -40*pi/180;
pitch_ub = 40*pi/180;

yaw_lb = -pi;
yaw_ub = pi;

thrust_lb = 10000;
thrust_ub = 60000;
