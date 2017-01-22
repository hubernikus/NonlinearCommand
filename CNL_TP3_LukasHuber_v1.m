%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Systeme Multivariables II
%   TP 3
%   
%   Created: basile.graf@epfl.ch
%   Modified: Lukas Huber (lukas.huber@epfl.ch)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables; close all; clc;

% Declarer les symboles utilises
syms x1 x2 x3 x4 u I J M G L K k real
x = [x1; x2; x3; x4]; % Creer le vecteur
n = length(x); % n=4

% QUESTIONs 1-6
% Definir les fonctions pour le crochet de Lie et la derive de Lie
LieBr = @(f1,f2) simplify(jacobian(f2,x)*f1-jacobian(f1,x)*f2);
LieDer = @(f,v) simplify(jacobian(v,x)*f);


% QUESTION 1
% Definir les champs de vecteurs f et g
k_q = K + (x1-x3)^2;
f = [x2;
    -1/I*(M*G*L*sin(x1)+k_q*(x1-x3));
    x4;
    1/J*k_q*(x1-x3)];

g=[0; 0; 0; 1/J];


% QUESTION 2
% Construire les distributions C et Cbar
Cbar = [g, LieBr(f,g), LieBr(f,LieBr(f,g))];
C = [Cbar, LieBr(f,LieBr(f,LieBr(f,g)))];

% Verifier l'accessibilite (rang de C)
fprintf('Acessibility check: \n')
fprintf('dim x = %d, rang C = %d \n\n', n, rank(C))

% Verifier l'involutivite de Cbar
fprintf('Involutivity check:\n')
check1 = det([Cbar, LieBr(g,LieBr(f,g))]);
check2 = det([Cbar, LieBr(g,LieBr(f,LieBr(f,g)))]);
check3 = det([Cbar, LieBr(g,LieBr(f,LieBr(f,LieBr(f,g))))]);

fprintf('Values of the determinants are:\n det1=%d \n det2=%d \n det3=%d \n \n', ...
                                            check1, check2, check3)

                                        
% QUESTION 3
% Construire la forme qui annule Cbar
omega = null(Cbar')';

% QUESTION 4
% Integrer omeg "a la main" (facile!)
fprintf('Integral of omega:\n');
h = omega*x % mu=1

% Construire le changement de coordonnes z = Phi(x)
fprintf('Change of coordinates:\n')
z1=h;
z2= LieDer(f,z1);
z3= LieDer(f,z2);
z4= LieDer(f,z3);
z = [z1; z2; z3; z4]

% Gains sur le systeme lineaire
k = [1 4 6 4];

% QUESTION 5
% Fonction de bouclage pour le systeme non-lineaire:
v = -k*z;
alpha = LieDer(f,z4);
beta =  LieDer(g,z4);
u = 1/beta*(v-alpha)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Simulation

%Parametres - modify parameters
I=0.01;         %inertie du bras articule
J=0.001;       %inertie du premier axe de rotation
K=5;          %constante de rappel du ressort
M=0.1;          %Masse de la sortie
L=0.1;          %1/2 longueur de l objet en sortie
G=10;           %gravite terrestre
k=1;

% Substituer les valeurs numeriques des parametres dans les expressions
us=subs(u);
fs=subs(f);
gs=subs(g);

% Creer une fonction a partir de ces expressions
dynamique = matfunc(fs+gs*us,'vars',[x1 x2 x3 x4]); 

% Conditions initiales
x0 = [.1,-.1,.2,0.1];
tspan=[0 25];

% Simulation numerique du systeme boucle
[t,xx] = ode45(@(t,x) dynamique(x(1),x(2),x(3),x(4)),tspan,x0);

figure('Position',[0,0,1000,350]);
plot(t,xx); hold on; grid on;
title('Systeme boucle')
xlabel('t'); ylabel('Magnitude')
legend('x_1(t)', 'x_2(t)', 'x_  3(t)', 'x_4(t)');
print('CNL_TP3_systemeBoucle3','-dpng')