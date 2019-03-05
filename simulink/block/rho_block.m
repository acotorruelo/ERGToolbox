function rho_block(block)
setup(block)

function setup(block)
block.NumDialogPrms = 1;

block.NumInputPorts = 3;
block.NumOutputPorts = 1;

block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

%% Set block sample time to continuous
block.SampleTimes = [0 0];

%% Set the block simStateCompliance to default (i.e., same as a built-in block)
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('Outputs',@Output);
block.RegBlockMethod('SetInputPortDimensions',@SetInputPortDimensions);
block.RegBlockMethod('Start',@Start);

function SetInputPortDimensions(block,idx,di)
block.InputPort(idx).Dimensions=di;
if idx==1||idx==2
    block.OutputPort(1).Dimensions=di;
end

function Start(block)
erg = block.DialogPrm(1).Data;
if ~isa(erg, 'ERGController')
    me = MSLException(block.BlockHandle, message('The first parameter must be an ERGController object'));
    throw(me);
end
 
function Output(block)
erg = block.DialogPrm(1).Data;
r=block.InputPort(1).Data;
v=block.InputPort(2).Data;
x=block.InputPort(3).Data;
block.OutputPort(1).Data=erg.NF(r,v,x);