clear
close all

%% SYSTEM DEFINITION

w=10;
xi=1.3;
A=kron(eye(2),[0 1;0 0]);
B=kron(eye(2),[0;1]);
C=kron(eye(2),[1 0]);
F=kron(eye(2),[-w^2 -2*xi*w]);
G=kron(eye(2),w^2);

%% SIMULATION PARAMETERS
x0=[-.7 0 -.7 0]';
Tsim=10;
Ts=1e-3;
t=0:Ts:Tsim;
    
ref=[1.75 -.25]';

r=kron(ones(size(t)),ref);

r=timeseries(r,t);

%% REGION PARAMETERS

M1=[1 0 0 0 0 0;-1 0 0 0 0 0;0 0 1 0 0 0;0 0 -1 0 0 0];
Mplot=[1 0;-1 0;0 1;0 -1];
M2=M1;
b1=[1 1 1 1]';
b2=[1.5 0 1.5 0]';
b3=[3 0 -.75 1.5]';
b4=[2 -1.25 1 1]';


sq1=ERGregion(M1,b1);
sq2=ERGregion(M2,b2);
sq3=ERGregion(M1,b3);
sq4=ERGregion(M2,b4);

Csq=sq1|sq2;
Csq2=sq3|sq4;

CSQ=Csq|Csq2;

sys=ERGsys(A,B,C,zeros(2,2),F,G);
sys=sys.setRegion(CSQ);
sys=sys.calculateQuadraticRegionalLyapunov;

s1=Polyhedron(M1,b1);
s2=Polyhedron(M1,b2);
s3=Polyhedron(M1,b3);
s4=Polyhedron(M1,b4);

s1plot=Polyhedron(Mplot,b1);
s2plot=Polyhedron(Mplot,b2);
s3plot=Polyhedron(Mplot,b3);
s4plot=Polyhedron(Mplot,b4);

kappa=1000;
eta=1e-3;
%% RUN THE SIMULATION
simout=sim('ex02_multiregion_sim.slx','StopTime',num2str(Tsim),'Solver','ode23s');

%% PLOT RESULTS
plot(s1plot,'Color',[0    0.4470    0.7410],'Alpha',.7)
hold on
plot(s2plot,'Color',[0.8500    0.3250    0.0980],'Alpha',.7)
plot(s3plot,'Color',[0.9290    0.6940    0.1250],'Alpha',.7)
plot(s4plot,'Color',[0.4940    0.1840    0.5560],'Alpha',.7)
f1=plot(squeeze(simout.xh.data(1,1,:)),squeeze(simout.xh.data(3,1,:)),'k--','LineWidth',2);
f2=plot(squeeze(simout.vh.data(1,1,:)),squeeze(simout.vh.data(2,1,:)),'b-.','LineWidth',2);
legend([f1 f2],'y','v')