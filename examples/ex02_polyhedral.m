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

s1=Polyhedron(M1,b1);
s2=Polyhedron(M1,b2);
s3=Polyhedron(M1,b3);
s4=Polyhedron(M1,b4);

POLY=s1|s2|s3|s4;

erg=ERGController(A,B,C,zeros(2,2),F,G);
erg.addPolyhedralConstraint(POLY,'sdpt3');

s1plot=Polyhedron(Mplot,b1);
s2plot=Polyhedron(Mplot,b2);
s3plot=Polyhedron(Mplot,b3);
s4plot=Polyhedron(Mplot,b4);

kappa=1000;
%% RUN THE SIMULATION
simout=sim('sim_model.slx','StopTime',num2str(Tsim),'Solver','ode23s','AbsTol','1e-5','RelTol','1e-3');

%% PLOT RESULTS
plot(s1plot,'Color',[0    0.4470    0.7410],'Alpha',.7)
hold on
plot(s2plot,'Color',[0.8500    0.3250    0.0980],'Alpha',.7)
plot(s3plot,'Color',[0.9290    0.6940    0.1250],'Alpha',.7)
plot(s4plot,'Color',[0.4940    0.1840    0.5560],'Alpha',.7)

f1=plot(squeeze(simout.xh.data(1,1,:)),squeeze(simout.xh.data(3,1,:)),'k--','LineWidth',2);
f2=plot(squeeze(simout.vh.data(1,1,:)),squeeze(simout.vh.data(2,1,:)),'b-.','LineWidth',2);
f3=plot(x0(1),x0(3),'ro','MarkerFaceColor','red');
f4=plot(ref(1),ref(2),'go','MarkerFaceColor','g');
legend([f1 f2 f3 f4],'y','v','x_0','r')
xlabel('x_1')
ylabel('x_3')