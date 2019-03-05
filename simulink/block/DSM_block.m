function DSM_block(block)
setup(block);

function setup(block)

% Register the number of ports.
block.NumInputPorts  = 2;
block.NumOutputPorts = 1;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override the input port properties.
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;
% Override the output port properties.
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).Dimensions = 1;

% Register the parameters.
block.NumDialogPrms     = 1;
block.DialogPrmsTunable = {'Nontunable'};

% Set up the continuous states.
block.NumContStates = 0;

block.SampleTimes = [0 0];

block.SetAccelRunOnTLC(false);
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
block.RegBlockMethod('Start',@Start);
block.RegBlockMethod('Outputs', @Outputs);
%endfunction

function SetInpPortDims(block, idx, di)

block.InputPort(idx).Dimensions = di;
block.OutputPort(1).Dimensions  = 1;

%endfunction

function Start(block)
sys = block.DialogPrm(1).Data;
if ~isa(sys, 'ERGController')
    me = MSLException(block.BlockHandle, message('The parameter must be an ERGController object'));
    throw(me);
end
%endfunction


function Outputs(block)
a = block.DialogPrm(1).Data;
x = block.InputPort(1).Data;
v = block.InputPort(2).Data;
block.OutputPort(1).Data = a.DSM(x,v);

%endfunction
