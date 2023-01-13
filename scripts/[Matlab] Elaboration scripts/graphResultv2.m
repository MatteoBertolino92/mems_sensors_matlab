clear all
close all
clc

fprintf('IMPORTANTE Aprire dapprima il file del sensore vicino (sensore 2) e poi quello del sensore lontano (sensore 1)\n')
ndscan = input('Numero di scan angolari -> ');
hscan  = input('Numero di quote scansite -> ');

%%%%%%%%%%%%%%%%%% INPUT nello script %%%%%%%%%%%%%%%%%%
R = 5; %Raggio carota
Fang = floor(360/ndscan); %Angolo di scan
fattint = 1; %fattore di interpolazione, non modificare
scalaq = 2;  %scala per la quiver
scala = 150; %fattore di riduzione grafica per la scatter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

itheta = (0:ndscan-1);%vettore con il numero di scan angolari
nth = length (itheta); %dovrebbe coincidere con ndscan
theta = 2*pi*Fang*itheta/360; %vettore riga, 36 colonne, con gli angoli delle componenti
thetaq = repmat(theta', hscan, 1);

%Bisogna rappresentare i nostri dati distribuiti su un cilindro, per il
%quale z è l'altezza, mentre x e y le componenti che delimitano la
%circonferenza. Nel codice seguente, z0 è un vettore che include tutte le
%altezze scansite, da 1 a 11, scalate di un fattore 100. 
%z0= (1:hscan)/100; %vettore riga da 11 colonne.
z0= (18-hscan+1:18)/100;
z0=fliplr(z0); 
R=R/100;

%Idem il R, diventa R/100. La quiver vuole un punto di applicazione e le componenti... le
%componenti sono quelle che calcoliamo noi (u, v, w), mentre i punti di
%applicazione sono dati dal raggio per il seno e coseno dell'angolo utilizzato per
%quel valore
x0 = R*cos(theta); %vettori riga da 36 colonne.
y0 = R*sin(theta);

%La quiver vuole componenti e punti di applicazioni, ma essendo questi ultimi 36, occorre replicarli per un
%numero di volte pari ad hscan!
x = repmat (x0', hscan, 1); %repmat prende x0 in ingresso, lo trasforma in un vettore colonna, lo replica per 11 righe e 1 colonna.
y = repmat (y0', hscan, 1);
za = repmat (z0, ndscan, 1); % ho una serie di righe uguali: 1 2 3 4 5 ecc
z=reshape (za, ndscan*hscan, 1); %incolonno le righe

i=1;
[u1, v1, w1, uq1, vq1, wq1, um1, vm1, wm1, cucu1, cucv1, cucz1, m3d1, modm1, uMap1, vMap1, wMap1, uxgq1, uygq1] = disegnaGrafici (ndscan, hscan, i, thetaq, x, y, z, scalaq, scala, x0, y0, z0, nth);
i=2;
[u2, v2, w2, uq2, vq2, wq2, um2, vm2, wm2, cucu2, cucv2, cucz2, m3d2, modm2, uMap2, vMap2, wMap2, uxgq2, uygq2] = disegnaGrafici (ndscan, hscan, i, thetaq, x, y, z, scalaq, scala, x0, y0, z0, nth);

%Ora che ho tutti i dati, calcolo il gradiente
m3d=sqrt((u1-u2).^2+(v1-v2).^2+(w1-w2).^2);
calcoloGradiente(uq1-uq2, vq1-vq2, wq1-wq2, x, y, z, thetaq, scalaq, u1-u2, v1-v2, w1-w2, x0, y0, z0, hscan, ndscan, nth, m3d, uxgq1-uxgq2, uygq1-uygq2);