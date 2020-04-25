

sigma=0.046;
Cdp=0.015;
g=9.81;
W=4309*g;
T=W;
rho=1.225;
Vt=227.381;
R=7.32;
k=1.15;
vih=sqrt(W/(2*rho*pi*R^2)); % induced velocity in hover 
Pid=W*vih;
V= 0:0.01:120;
mi=V./Vt;
Aeq=1.885932; % equivalent flat plate area 


Pp_h = sigma*Cdp/8*rho*(Vt)^3*pi*R^2; %profile drag power in hover calculated using BEM
Pi_h = k*T*vih; % induced power in hover calculated using BEM
Phov=Pp_h+Pi_h; % powwwer in hover using BEM 
F=Pid/Phov;

mi_plus_one=ones(1,length(mi));

for i=1:length(mi)
    mi_plus_one(i) = 1+mi(i)^2;
end


Pp_ff= Pp_h.*mi_plus_one; %profile power forward flight 
Pd_ff= sigma*Cdp/4*rho*Vt^3*pi*R^2*1.825.*mi.^2; % rotor drag power forward flight. Note 1.825 is needed to get to the bennet approx when summing Pp_ff and Pd_ff 
Ppd_ff= Pp_ff + Pd_ff;
%Ppd_ff= Pp_h.*(1+4.65.*mi.^2);

Dpar = 0.5*Aeq*rho*V.^2; % fuselage parasite drag 
Ppar_ff = Dpar.*V; % fuselage parasite power



%------ attempt to calculate the induced velocity in forward flight using iterative process -------


error = 1;
Vbar = V./vih;
vi_bar = 1;
vi2list=ones(1,length(V)); % list containing the normalised induced velocities in forward flight 

  
    for i = 1 : length(Vbar)
        
    while error>0.0001  
        
    sin_alpha = Dpar(i)/W;
    alpha = asin(sin_alpha);
    vi2_bar = 1/((Vbar(i)*cos(alpha))^2+(Vbar(i)*sin(alpha)+vi_bar)^2);
   
    error = sqrt(vi2_bar) - vi_bar;
    vi_bar = sqrt(vi2_bar); 
    
    end
    
    vi2list(i) = vi2_bar;
    error = 1;
    end
    
    vi_ff = sqrt(vi2list);
    vi_ff =vi_ff.*vih; % induced velocity in forward flight
    V_R = (W./((2*rho*R^2).*vi_ff));%vectorial sum of V and vi_ff
    T = 2*rho*pi*R^2.*V_R.*vi_ff; % thrust force in forward flight 
    Pi_ff = k.*T.*vi_ff; %power induced forward flight 
    
 
 

vi_ff = sqrt(vi2list); 
%plot(Vbar,vi_ff);
%pause
vi_ff =vi_ff.*vih; % induced velocity in forward flight
V_R = (W./((2*rho*R^2).*vi_ff));%vectorial sum of V and vi_ff
T = 2*rho*pi*R^2.*V_R.*vi_ff; % thrust force in forward flight 
Pi_ff = k.*T.*vi_ff; %power induced forward flight 

Ptot_ff = Ppd_ff + Ppar_ff + Pi_ff; %total power forward flight 


%plot profile power + drag power and the sum of the induced power 
%plot(V,Pp_ff,'linewidth',1.3)


%plot(V,Pd_ff,'linewidth',1.3)
pl1 = plot(V, Ppd_ff./1e3,'linewidth',1.3);
ax = ancestor(pl1, 'axes');
ax.YAxis.Exponent = 0;
ytickformat('%d');

hold on 
pl2 = plot(V, Ppar_ff./1e3,'linewidth',1.3);
ax = ancestor(pl2, 'axes');
ax.YAxis.Exponent = 0;
ytickformat('%d');

pl3 = plot(V,Pi_ff./1e3,'linewidth',1.3);
ax = ancestor(pl3, 'axes');
ax.YAxis.Exponent = 0;
ytickformat('%d');

pl4 = plot(V,Ptot_ff./1e3,'linewidth',1.5);
ax = ancestor(pl4, 'axes');
ax.YAxis.Exponent = 0;
ytickformat('%d');

xlabel('V [$\frac{m}{s}$]','Interpreter','latex','FontSize',15);
ylabel('Power [kW]','Interpreter','latex','FontSize',15);


legend('Rotor profile drag power + Rotor drag power','Fuselage parasite power','Induced power','Total power','Interpreter','latex','Fontsize',10);
hold off
pause




