%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   Syst�me Multivariables II
%   TP 3
%
%   Supprimer les %%% et remplacer les ...... par ce qu'il convient!
%
%   basile.graf@epfl.ch
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear all
close all

% D�clarer les symboles utilis�s
syms x1 x2 x3 x4 u I J M G L K k real

% QUESTIONs 1-6
% D�finir les fonctions pour le crochet de Lie et la dd�riv�e de Lie
%%% LieBr = @(f1,f2) simplify(.......);
%%% LieDer = @(f,v) simplify(.......);

% QUESTION 1
% D�finir les champs de vecteurs f et g
%%% f=[
%%%    ........
%%% ]

%%% g=[
%%%    ........
%%% ]


% QUESTION 2
% Construire les distributions C et Cbar
%%% Cbar = .......
%%% C = ........

% V�rifier l'accessibilit� (rang de C)
'Acessibility check:'
%%% .......

% V�rifier l'involutivit� de Cbar
'Involutivity check:'
%%% .......
%%% .......
%%% .......


% QUESTION 3
% Construire la forme qui annule Cbar
%%% .......

% QUESTION 4
% Int�grer omeg "� la main" (facile!)
'Integral of omega:'
%%% h= ......

% Construire le changement de coordonn�es z = Phi(x)
'Change of coordinates:'
%%% z1=h;
%%% z2= ......
%%% z3= ......
%%% z4= ......
%%% z = [z1; z2; z3; z4]

% Gains sur le syst�me lin�aire
k = [1 4 6 4];

% QUESTION 5
% Fonction de bouclage pour le syst�me non-lin�aire:
%%% alpha =  ......
%%% beta =  ......
%%% u =  ......

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Simulation

%Parametres
I=0.1;         %inertie du bras articule
J=0.001;       %inertie du premier axe de rotation
K=0.1;          %constante de rappel du ressort
M=0.1;          %Masse de la sortie
L=0.3;          %1/2 longueur de l objet en sortie
G=10;           %gravite terrestre
k=1;

% Substituer les valeurs num�riques des param�tres dans les expressions
%%% us=subs(u);
%%% fs=subs(f);
%%% gs=subs(g);

% Creer une fonction � partir de ces expressions
%      dynamique = matlabfunction(fs+gs*us,'vars',[x1 x2 x3 x4]); % MATLAB >= R2008b
%%% dynamique = matfunc(fs+gs*us,'vars',[x1 x2 x3 x4]);     % MATLAB < R2008b

% Conditions initiales
%%% x0 = [.1,-.1,.2,0.1];
%%% tspan=[0 25];

% Simulation num�rique du syst�me boucl�
%%% [t,xx] = ode45(@(t,x) dynamique(x(1),x(2),x(3),x(4)),tspan,x0);

%%% plot(t,xx)
%%% title('Syst�me boulc�')
%%% xlabel('t')
%%% ylabel('x_1(t), x_2(t), x_3(t), x_4(t)')