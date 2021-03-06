P1_span = linspace(pid.P1.Minimum,pid.P1.Maximum,50+1);
P2_span = linspace(pid.P2.Minimum,1e6,20+1)';
[P1,P2] = ndgrid(P1_span,P2_span');

Tp_span = pid.C3*(P2_span' + pid.R8);
% pole1_span = 1./Tp_span

Pole1 = 1./( pid.C3*(P2_span+pid.R8));

Kdc = 1./(pid.C4*(P1+pid.R6));
Kdc_min = min(Kdc)';
Kdc_max = max(Kdc)';

Kp = (pid.R8 + P2_span)./ pid.R5;

Tz = (P2+pid.R8).*( pid.C4.*(P1+pid.R6) + pid.R5*pid.C3 ) ./ pid.R5;
Zero1_max = 1./min(Tz)';
Zero1_min = 1./max(Tz)';

tab = table(P2_span,Kp,Pole1,Kdc_min,Kdc_max,Zero1_min,Zero1_max)
tabA = table2array(tab);
columnLabels = {'$P_2$','$K_p$','Pole 1','$K_{dc,min}$','$K_{dc,max}$','Zero min','Zero max'};
matrix2latex(tabA, 'PIDtab.tex', 'columnLabels', columnLabels, 'alignment', 'c', 'format', '%-6.2f');