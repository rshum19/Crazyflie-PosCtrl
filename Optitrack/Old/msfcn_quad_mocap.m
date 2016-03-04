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
function msfcn_quad_mocap(block)
% 
% The setup method is used to setup the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.  
%   
% clear the command screen
clc;
global velocity_data position_data var
   var = 0
   position_data = 0;
   velocity_data = 0;
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
  % 1. Number of the quadrotor in view
  % Output Port:
  % not using any quaternion for now
  % 1. position of a single quadrotor
  % 2. velocities of the quadrotors
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 3; 
  
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  % Override the input port properties.
  % the number of quadrotor 
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity = 'Real';
  block.InputPort(1).Dimensions = 1;
  
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
  block.RegBlockMethod('Update', @Update);

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
    block.NumDworks = 7;
    
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
    
    % current state
    block.Dwork(5).Name            = 'current_state';
    block.Dwork(5).Dimensions      = 3;
    block.Dwork(5).DatatypeID      = 0;      % double
    block.Dwork(5).Complexity      = 'Real'; % real
    block.Dwork(5).UsedAsDiscState = false;
    
    % the frame time for each data
    block.Dwork(6).Name = 'data_time';
    block.Dwork(6).Dimensions = 5;
    block.Dwork(6).DatatypeID = 0;
    block.Dwork(6).Complexity = 'Real';
    block.Dwork(6).UsedAsDiscState = false;
   
    % the current roll angles from the motion capture system
    block.Dwork(7).Name = 'current_roll';
    block.Dwork(7).Dimensions = 3;
    block.Dwork(7).DatatypeID = 0;
    block.Dwork(7).Complexity = 'Real';
    block.Dwork(7).UsedAsDiscState = false;
    

    
    % Register all tunable parameters as runtime parameters.
    block.AutoRegRuntimePrms;
    
    % =====================================================================
    % .Net application initialization
    % add the library path 
    dllPath = fullfile('C:','NatNet_SDK_2.6','lib','x64','NatNetML.dll');
    assemblyInfo = NET.addAssembly(dllPath);

    % create client for data extraction 
    % this client is used during the whole simulation loop
    global theClient;
    theClient = NatNetML.NatNetClientML(1); % 1 for unicasting
    HostIP = char('127.0.0.1');
    flag = theClient.Initialize(HostIP, HostIP);
    % test whether the initialization is successful or not
    assert(flag == 0);
    % =================================================================
%endfunction

% % set-up initial conditions 
function InitializeConditions(block)
% initial condition setup
%endfunction

function Start(block)
    % display the command on the screen
    disp('Motion Capture System has started');
    % initial conditions for the internal discrete states
    block.Dwork(1).Data = zeros(3,1);
    block.Dwork(2).Data = zeros(3,1);
    block.Dwork(3).Data = zeros(3,1);
    block.Dwork(4).Data = zeros(3,1);
    block.Dwork(5).Data = zeros(3,1);
    block.Dwork(6).Data = [0;1;2;3;4];
%endfunction

% function WriteRTW(block)
%   
%     block.WriteRTWParam('matrix', 'M',    [1 2; 3 4]);
%     block.WriteRTWParam('string', 'Mode', 'Auto');
%    
% %endfunction

function Outputs(block)
% using interpolation for estimating the translational velocity
% could add spline later for better results
 
option = 'interp';

switch option
    case 'interp'
        % using Lagrangian formula for computing the time-derivative
        % output the position first
        cPosition = block.Dwork(5).Data;
        Time = block.Dwork(6).Data
        % center the time series
        Time = Time - Time(1)
        
        % the polynomial degree for interpolation
        N = 1;
        
        % the sampled point for computing the time derivative
        M = 3;
        
        % ----------------------------------------------
        % Could be much faster by using linear matrix multiplication
        % ----------------------------------------------
        % line all the data in seprate vectors
        X = [block.Dwork(1).Data(1);block.Dwork(2).Data(1);
            block.Dwork(3).Data(1);block.Dwork(4).Data(1);
            block.Dwork(5).Data(1)];
        Y = [block.Dwork(1).Data(2);block.Dwork(2).Data(2);
            block.Dwork(3).Data(2); block.Dwork(4).Data(2);
            block.Dwork(5).Data(2)];
        Z = [block.Dwork(1).Data(3);block.Dwork(2).Data(3);
            block.Dwork(3).Data(3); block.Dwork(4).Data(3);
            block.Dwork(5).Data(3)];
      block.Dwork(6).Data(1); % Real Time
%       display('Roll');
%       block.Dwork(7).Data(1)*180/pi % Roll in radians 
     
        % turn off the warning
        warning('off');
        
        % get the polynomial coefficient
        PX = polyfit(Time, X, N);
        PY = polyfit(Time, Y, N);
        PZ = polyfit(Time, Z, N);
        
        % get the corresponding time derivative
        dPX = polyder(PX);
        dPY = polyder(PY);
        dPZ = polyder(PZ);
        
        % get the value at the sampled point
        cVelocity = [polyval(dPX, Time(M));
                    polyval(dPY, Time(M));
                    polyval(dPZ, Time(M))];
                
%          display('position X Y Z');
         cPosition = cPosition';
%          display('velocity X Y Z');
         cVelocity = cVelocity';
         cPosition(1)
         
         
         
        % output the corresponding position and velocity
        block.OutputPort(1).Data = cPosition; 
        block.OutputPort(2).Data = cVelocity;
        block.OutputPort(3).Data = block.Dwork(7).data;
    case 'spline' 
        % use cubic spline to compute the quadrotor's velocity
        % could be developed for further smooth 
        
end

function Update(block)
    % update the previous stored data
    for i = 1:4
        % shift forward the sampled data
        block.Dwork(i).Data = block.Dwork(i+1).Data;
        block.Dwork(6).Data(i) = block.Dwork(6).Data(i+1);
    end
    [QuadPos, All_angle, FrameTime] = getFrameData(block);
    block.Dwork(5).Data = double(QuadPos);
    block.Dwork(6).Data(4) = FrameTime;
    block.Dwork(7).Data = All_angle; % in radian
    
%endfunction

function [QuadPos,All_angle, FrameTime] = getFrameData(block)
global theClient;

% get the latest frame of data by direct calling back
frameData = theClient.GetLastFrameOfData();

% track the rigid body number
quadNum = block.InputPort(1).Data;

% get the corresponding rigid body data structure
rbData = frameData.RigidBodies(quadNum);
FrameTime = frameData.fLatency;

% get the current angles
q = quaternion( rbData.qx, rbData.qy, rbData.qz, rbData.qw );
qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
q = mtimes(q, qRot);
angles = EulerAngles(q,'zyx');
All_angle = -angles;

% get the corresonding position and convert it to that in reference frame
QuadPos = [rbData.x; -rbData.z; rbData.y];

%endfunction
    
function Terminate(block)
% uninitialize the assembly object
global theClient;
theClient.Uninitialize();
disp('The motion capture system is closed...');


 