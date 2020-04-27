%% Power hover
clc
g=9.81;
W=4309*g;
T=W;
Vt=227.381;
R=7.32;
sigma=0.046;
Cdp=0.015;
rho = 1.225
V= 0:0.01:120;
omega = 33.9292;
type_main = 1;

[vih,Pp_h,Phov]= power_hover(sigma,Cdp,rho,R,T,V,Vt);


%% Power main rotor 

[vi_ff,Ptot_ff,Ppd_ff,Pi_ff] = Power(T,V,Vt,R,sigma,Cdp,vih,Pp_h,type_main);

%% Power tail rotor 

Vt_tr=224.333;
R_tr=1.2954;
sigma_tr=0.105;
Cdp_tr=0.0085;
type_tr = 2;
l_tr = 9.1444;
T_tr = Phov/(omega*l_tr);

[vi_ff_tr,Ptot_ff_tr,Ppd_ff_tr,Pi_ff_tr] = Power(T_tr,V,Vt_tr,R_tr,sigma_tr,Cdp_tr,vih,Pp_h,type_tr);

%% Summing the all the contributions 

Ptotal = Ptot_ff + Ptot_ff_tr;

pl1 = plot(V, Ptotal./1e3,'linewidth',1.3);
ax = ancestor(pl1, 'axes');
ax.YAxis.Exponent = 0;
ytickformat('%d');
    
xlabel('V [$\frac{m}{s}$]','Interpreter','latex','FontSize',15);
ylabel('Power [kW]','Interpreter','latex','FontSize',15);
    
legend('Total power required','Interpreter','latex','Fontsize',10);

hold on 

index_maxend = find(Ptotal == min(Ptotal));
V_maxend = V(index_maxend); % maximum endurance speeed

YoverX = Ptotal./V;
index_maxrange = find(YoverX == min(YoverX));
V_maxrange = V(index_maxrange);

end_mark = min(Ptotal).*(ones(1,length(V)));
range_mark = min(YoverX).*V;

plot(V,end_mark./1e3);
plot(V,range_mark./1e3);
