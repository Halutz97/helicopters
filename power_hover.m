function [vih,Pp_h,Phov]= power_hover(sigma,Cdp,rho,R,T,V,Vt,k)

clc
k = 1.15;
W = T;
vih=sqrt(W/(2*rho*pi*R^2)); % induced velocity in hover 
Pid=W*vih;
mi = V./Vt;

Pp_h = sigma*Cdp/8*rho*(Vt)^3*pi*R^2; %profile drag power in hover calculated using BEM
Pi_h = k*T*vih; % induced power in hover calculated using BEM
Phov=Pp_h+Pi_h; % powwwer in hover using BEM 
F=Pid/Phov;


end