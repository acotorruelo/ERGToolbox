close all
clear

%% DEFINITION OF THE SYSTEM
w=10;
z=.7;
A=[0 1;0 0];
B=[0;1];
C=[1 0];
F=[-w^2 -2*z*w];
G=w^2;
x0=[0 0]';

%% SIMULATION PARAMETERS
Tsim=6; %Simulation time
Ts=1e-3; %Time step
t=0:Ts:Tsim;
ref=42; %Reference
r=ref*ones(size(t));
r=timeseries(r,t);

%% CREATION OF THE ERGSYS OBJECT

sys=ERGsys(A,B,[0 1],0,F,G); %Creation of an instance of ERGsys
sys=sys.addStateConstraint([1 0]',42); %Adding state constraints
sys=sys.addInputConstraint('u',100); %Adding input constraints
sys=sys.addInputConstraint('l',-100);
sys=sys.calculateQuadraticLyapunov; %Calculate the Lyapunov functions associated to the constraints
kappa=1000; %Gain
eta=1e-3; %Eta parameter
simout=sim('ex01_sim.slx','StopTime',num2str(Tsim),'Solver','ode23s','AbsTol','1e-9','RelTol','1e-7'); %Simulate

%% PLOTTING

figure
subplot(211)
plot(simout.xh.time,squeeze(simout.xh.data(1,:))','LineWidth',2)
hold on
title('Output and applied reference','FontSize',12)
xlabel('Time (s)','FontSize',12)
ylabel('y, v','FontSize',12)
plot(simout.vh.time,squeeze(simout.vh.data),'LineWidth',2)
legend('y','v','Location','best')
hold off
grid on
subplot(212)
plot(simout.uh.time,squeeze(simout.uh.data),'LineWidth',2)
title('Input action','FontSize',12)
xlabel('Time (s)','FontSize',12)
ylabel('u','FontSize',12)
grid on