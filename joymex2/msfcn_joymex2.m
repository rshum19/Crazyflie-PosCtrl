function msfcn_joymex2(block)
%MSFCN_JOYMEX2 Let you use a  PS3 controller as input device
%   This S-function is based on the joymex2 function created by Martijn Aben
%   (http://escabe.org/joomla/index.php/7-projects/matlab/1-joymex2
%
%   It should be noted that any joystick that is configurable in linux
%   can be used with this S-function block. However, the output mapping may
%   be wrong. 
%
%   Paramenter Input:
%   Joystick id: usually 0

%   Outputs:
%       - Buttons Vector:
%           1: Select           11: L2
%           2: L3               12: R2
%           3: R3               13: Triangle
%           4: Start            14: Circle
%           5: Up               15: X
%           6: Right            16: Square
%           7: Down             17: PS
%           8: Left
%           9: L1
%           10:R1
%
%       - Axes Vector:
%           1: LStick horizontal
%           2: LStick vertical
%           3: RStick horizontal
%           4: RStick vertical
%           5: Roll Axis
%           6: Pitch Axis
%           7: Yaw Axis

%   Copyright 2015 Roberto Shu

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 0;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompPortInfoToDefaults;

% Override output port properties
% OutputPort(1) : Joystick axes values
block.OutputPort(1).Dimensions  = 7;
block.OutputPort(1).DatatypeID  = 4; % int16
block.OutputPort(1).Complexity  = 'Real';

% OutputPort(2) : Joystick button values
block.OutputPort(2).Dimensions  = 17;
block.OutputPort(2).DatatypeID  = 3; % uint8
block.OutputPort(2).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 1;

% Register sample times
%  [-1, 0]               : Inherited sample time
block.SampleTimes = [-1 0];

% Specify the block simStateCompliance. The allowed values are:
%    'DefaultSimState', < Same sim state as a built-in block
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%

%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
function Start(block)

    joyID = block.DialogPrm(1).Data;
    joymex2('open',0)

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

    joyID = block.DialogPrm(1).Data;
    buttonsStruct = joymex2('query',joyID);
    
    block.OutputPort(1).Data = buttonsStruct.axes(1:7); % joysticks -100 - 100%
    block.OutputPort(2).Data = buttonsStruct.buttons; % Buttons 0 or 1
%end Outputs

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)
    clear joymex2
%end Terminate

