clear all
close all
clc

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
% 
% i=1;
% [u1, v1, w1, uq1, vq1, wq1, um1, vm1, wm1, cucu1, cucv1, cucz1, m3d1, modm1, uMap1, vMap1, wMap1] = disegnaGrafici (ndscan, hscan, nth, i);
% i=2;
% [u2, v2, w2, uq2, vq2, wq2, um2, vm2, wm2, cucu2, cucv2, cucz2, m3d2, modm2, uMap2, vMap2, wMap2] = disegnaGrafici (ndscan, hscan, nth, i);
for i=1:2   
    [nome, path]=uigetfile('*.txt', ['Open file called *scan*.txt per il sensore ', num2str(i)]);
    fp = fopen([path, nome], 'r');
    M  = fscanf(fp, '%g %*c %g %*c %g', [3, inf]);
    M = M';
    fclose(fp);

    u = M(:, 1); %In w0, u0 e v0 ci sono rispettivamente le 396 medie di x, y, z. 
    v = M(:, 2); %Ciascun valore è la media su 50 valori di una misurazione, ad altezza e angolo fissate.
    w = M(:, 3); %Ogni 36 righe gli angoli si ripetono..! Sono vettori colonna di 396 elementi = 36*11.
    
    %u= componente verticale
    %v= componente tangenziale
    %w= componente radiale

    %Rotazione di assi per quiver
    uq= v.*sin(thetaq) - w.*cos(thetaq);
    vq=-v.*cos(thetaq) - w.*sin(thetaq);
    wq=-u;

    %uq = componente radiale
    %vq = componente tangenziale
    %wq = componente verticale
    um= mean(uq);
    vm= mean (vq);
    wm = mean (wq);

    figure
    quiver3 (x, y, z, uq-um, vq-vm, wq-wm, scalaq) %Rimuovo la componente continua
    title(['Componenti per sensore ', num2str(i)])
    axis equal
    axis square
    grid on

    figure
    wq=wq-wq;
    quiver3 (x, y, z, uq-um, vq-vm, wq, scalaq) %Rimuovo la componente continua
    title(['Radiali e tangenziali su piano orizzontale per sensore ', num2str(i)])
    axis equal
    axis square

    %La scatter disegna dei pallini pieni, con il modulo colorato
    m3d=sqrt(u.^2+v.^2+w.^2);
    figure
    scatter3(x,y,z, m3d/scala, m3d,'filled');
    title(['Modulo con lo scatter3 per sensore ', num2str(i)])
    axis equal
    axis square

    %Disegnare le mappe di modulo, componente radiale, tangenziale, verticale,
    %con la funzione SURF. SURF vuole in ingresso surf(X,Y,Z,C), dove X, Y, Z
    %sono le nostre componenti non ruotare ---> u,v,w, e C è x la colorazione.
    %Nel caso del modulo, la colorazione è data dalla media, per la componente
    %radiale da u, la tangenziale da v, la verticale da w ecc... Io disegno un
    %cilindro e poi lo coloro in base a quello che voglio vedere!

    %Attenzione: la surf vuole che X, Y, Z, C siano tutte matrici della
    %medesima dimensione. cucu e cucv sono matrici 11x36, con le componenti
    %della circonferenza del cilindro. Sono ottenute a partire da x0 e y0,
    %vettori riga da 36 colonne.. Replico tali vettori 11 volte, e li
    %incolonno. Così per ogni altezza ho sempre lo stesso cerchio.
    %Coerentemente, cucz è una matrice 11x36, dove in ogni colonna c'è una
    %altezza diversa.
    cucu = repmat(x0, hscan, 1);     cucu(:,nth+1)=cucu(:,1);
    cucv = repmat(y0, hscan, 1);     cucv(:,nth+1)=cucv(:,1);
    cucz = repmat(z0', 1, nth);      cucz(:,nth+1)=cucz(:,1);
    modm = reshape(m3d',nth,hscan)'; modm(:, nth+1) = modm (:, 1); %la reshape incolonna la riga, per cui devo avere un vettore riga...
    uMap = reshape (w', nth, hscan)'; uMap(:, nth+1) = uMap(:, 1); %componente radiale per la mappa 
    vMap = reshape (v', nth, hscan)'; vMap(:, nth+1) = vMap(:, 1); %componente tangenziale per la mappa 
    wMap = reshape (u', nth, hscan)'; wMap(:, nth+1) = wMap(:, 1); %componente verticale per la mappa

    figure
    surf (cucu, cucv, cucz, modm, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Modulo per sensore ', num2str(i)])

    figure
    surf (cucu, cucv, cucz, uMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Radiale per sensore ', num2str(i)])

    figure
    surf (cucu, cucv, cucz, vMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Tangenziale per sensore ', num2str(i)])

    figure
    surf (cucu, cucv, cucz, wMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Verticale per sensore ', num2str(i)])

    figure
    uxgq=reshape(uq-um,nth,hscan)';    uxgq(:,nth+1)=uxgq(:,1);
    uygq=reshape(vq-vm,nth,hscan)';    uygq(:,nth+1)=uygq(:,1);

    for j=1:hscan
     %Prima freccia rossa
     subplot(4,4,j);
     quiver(cucu(j, 1:2), cucv(j, 1:2), uxgq(j, 1:2),uygq(j, 1:2), 'r', 'Linewidth', 1.25);
     hold on;
     subplot (4,4,j);
     quiver(cucu(j, 2:ndscan), cucv(j, 2:ndscan), uxgq(j, 2:ndscan), uygq(j, 2:ndscan), 'b', 'Linewidth', 1.3);
     hold on
     subplot(4,4,j); plot(x0,y0,'k','LineWidth', 1);
     axis equal
     grid on
     title(num2str(19-j));
    end 
end

%Ora che ho tutti i dati, calcolo il gradiente