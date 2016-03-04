% =========================================================
% Motion Capture System Simulink Block
% using s-function 
% Dynamic Robotic Labs
% Carnegie Mellon University
% =========================================================
% 1.0 version by Guofan Wu(gwu)
% Modified from Matlab template
% use previous 4 points for velocity estimation
% August 4th, 2014
% =========================================================
% 1.1 Version by Guofan Wu(gwu)
% Changes:
% include 5 points for velocity estimation
% with 2 sampled delay using interpolation
% using polynomial fit for tuning parameter
% =========================================================
% the velocity estimation only works for the sampling time 0.02
% =========================================================
function msfcn_parse_mocap_data(block)
% 
% The setup method is used to setup the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.  
%   
% clear the command screen
clc;
setup(block);
  
%endfunction

% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
% 
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)
   
  % Register the number of ports.
  % Input Port:
  %     1. Mocap position data frame [x,y,z]
  %     2. Mocap orientation data frame [qx qy qz qw];
  % Output Port:
  %     1. position
  %     2. linear velocities
  %     3. Orientation angles [roll,pitch,yaw];
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 3; 
  
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  % Mocap position data vector 
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity = 'Real';
  block.InputPort(1).Dimensions = 3;
  
  % Mocap orientation data quaternion
  block.InputPort(2).DatatypeID  = 0;  % double
  block.InputPort(2).Complexity = 'Real';
  block.InputPort(2).Dimensions = 4;
  
  % Override the output port properties.
  % all double type
  % output 1-3: positions
  % output 4-6: estimated velocity using interpolation
  % output 7: angles
  block.OutputPort(1).DatatypeID = 0;
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(1).Dimensions = 3;
  block.OutputPort(2).DatatypeID = 0;
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(2).Dimensions = 3;
  block.OutputPort(3).DatatypeID = 0;
  block.OutputPort(3).Complexity = 'Real';
  block.OutputPort(3).Dimensions = 3; 
  
%   % Register the parameters.
%   block.NumDialogPrms     = 3;
%   % set up the tunable parameters in 
%   block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable'};
  
%   % Set up the continuous states.
%   block.NumContStates = 0;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  % inherit the sampling time from input 
  block.SampleTimes = [-1 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  
  % Specify the block simStateCompliance. The allowed values are:
  %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
  %    'DefaultSimState', < Same SimState as a built-in block
  %    'HasNoSimState',   < No SimState
  %    'CustomSimState',  < Has GetSimState and SetSimState methods
  %    'DisallowSimState' < Errors out when saving or restoring the SimState
  block.SimStateCompliance = 'DefaultSimState';
  
  % -----------------------------------------------------------------
  % The MATLAB S-function uses an internal registry for all
  % block methods. You should register all relevant methods
  % (optional and required) as illustrated below. You may choose
  % any suitable name for the methods and implement these methods
  % as local functions within the same file.
  % -----------------------------------------------------------------
   
  
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
  
  % PostPropagationSetup:
  %   Functionality    : Set up the work areas and the state variables. You can
  %                      also register run-time methods here.
  %   C-Mex counterpart: mdlSetWorkWidths
  %
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

 
  % InitializeConditions:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlInitializeConditions
  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  % 
  % Start:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlStart
  %
  block.RegBlockMethod('Start', @Start);

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C-Mex counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);

  % 
  % Update:
  %   Functionality    : Call to update the discrete states
  %                      during a simulation step.
  %   C-Mex counterpart: mdlUpdate
  %
  %block.RegBlockMethod('Update', @Update);

  % 
 
  block.RegBlockMethod('Terminate', @Terminate);


% this one is necessary for the multiple output ports 
function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode = fd;
  block.OutputPort(2).SamplingMode = fd;
  block.OutputPort(3).SamplingMode = fd;
% %endfunction

    
function DoPostPropSetup(block)
    % computation of the current quad position and
    % its 1st order derivatives using previous three data
    block.NumDworks = 4;
    
    % state component
    % 1. x
    % 2. y
    % 3. z
    % previous 4 state
    block.Dwork(1).Name            = 'previous4_state';
    block.Dwork(1).Dimensions      = 3;
    block.Dwork(1).DatatypeID      = 0;      % double
    block.Dwork(1).Complexity      = 'Real'; % real
    block.Dwork(1).UsedAsDiscState = false;
    
    % previous 3 state
    block.Dwork(2).Name            = 'previous3_state';
    block.Dwork(2).Dimensions      = 3;
    block.Dwork(2).DatatypeID      = 0;      % double
    block.Dwork(2).Complexity      = 'Real'; % real
    block.Dwork(2).UsedAsDiscState = false;
    
    % previous 2 state
    block.Dwork(3).Name            = 'previous2_state';
    block.Dwork(3).Dimensions      = 3;
    block.Dwork(3).DatatypeID      = 0;      % double
    block.Dwork(3).Complexity      = 'Real'; % real
    block.Dwork(3).UsedAsDiscState = false;
    
    % previous 1 state
    block.Dwork(4).Name            = 'previous1_state';
    block.Dwork(4).Dimensions      = 3;
    block.Dwork(4).DatatypeID      = 0;      % double
    block.Dwork(4).Complexity      = 'Real'; % real
    block.Dwork(4).UsedAsDiscState = false;
    
    % Register all tunable parameters as runtime parameters.
    block.AutoRegRuntimePrms;
    
    % Initialize linear system for interpolation
    global A B C D;
    [A, B, C, D,] = num_der(2,5,3,0.01);
    C = C(2,:); D = D(2,:);

%endfunction

% % set-up initial conditions 
function InitializeConditions(block)
% initial condition setup
%endfunction

function Start(block)
    % display the command on the screen
    disp('The motion Capture System has started');
   
    % initial conditions for the internal discrete states
    block.Dwork(1).Data = zeros(3,1);
    block.Dwork(2).Data = zeros(3,1);
    block.Dwork(3).Data = zeros(3,1);
    block.Dwork(4).Data = zeros(3,1);
%endfunction

% function WriteRTW(block)
%   
%     block.WriteRTWParam('matrix', 'M',    [1 2; 3 4]);
%     block.WriteRTWParam('string', 'Mode', 'Auto');
%    
% %endfunction

function Outputs(block)
% using interpolation for estimating the translational velocity with 
 
global A B C D;

% Get input readings
currPos = block.InputPort(1).Data';
QuatOrnt = block.InputPort(2).Data;

% --------------------------------------------------
% numerical differentiation using interpolation based 
% on "num_der.m"
% N = 2; M = 5; m0 = 2; dt = 0.02; 

% Load previously saved position
prevPos = [block.Dwork(1).Data, block.Dwork(2).Data, block.Dwork(3).Data, block.Dwork(4).Data]';

% turn off the warning
warning('off');

% Estimate velocity
currVel = C*prevPos + D*currPos;

% Update Position
prevPos = A*prevPos + B*currPos;
for i = 1:size(prevPos,1)
    block.Dwork(i).Data = prevPos(i,:)';
end

% get the current angles
q = quaternion( QuatOrnt(1), QuatOrnt(2), QuatOrnt(3), QuatOrnt(4) );
qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
q = mtimes(q, qRot);
angles = EulerAngles(q,'zyx');
angleX = -angles(1)*180.0 / pi;
angleY = angles(2)*180.0 / pi;
angleZ = -angles(3)*180.0 / pi;

% output the corresponding position and velocity
block.OutputPort(1).Data = currPos; 
block.OutputPort(2).Data = currVel;
block.OutputPort(3).Data = [angleX; angleY; angleZ];



function Terminate(block)


disp('The motion capture system is closed...');


 