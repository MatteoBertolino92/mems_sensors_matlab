%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dati in ingresso 36 file contenenti le misure per una certa altezza, il %
% programma mi crea un file con media, dev standard, precisione di x, y,z %
% per quella altezza.                                                     %
% Si ripete per ciascuna quota, fino ad avere un descrittore per ogni     %
% altezza.                                                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

mag='M2';
i = 0;
j = 0;

quotain = 5;
ang=0;
OUT = zeros(36,11);
M = zeros (200, 200);
nq = input (' Numero di quote totali: ');
nf = input (' Numero posizioni angolari per ogni quota = numero di file: ');
c = input  (' Inserisci 0 per misurazioni a vuoto, 1 con la carota: ');
if c==0
    car = 'sp';
else
    car = 'cp';
end

D = zeros (nf*nq, 3);

%Chiedo il primo file per quella altezza

quota=5;
k=1;

for j=1:1:nq
    ang=0;
    disp( sprintf( 'Calcolo sui file a quota %d', quota) )
    [nome, path] = uigetfile ('*.txt', 'Seleziona il primo file per la quota corrente');
    newn = nome;
    for i=1:1:nf
        %Acquisisco le colonne X, Y, Z in 3 vettori colonna
        [newn, path];
        fp = fopen ([path, newn], 'r');
        line1=fgets(fp);
        M = fscanf (fp, '%g %g %g', [3 inf]);
        fclose (fp);
        
        M = M';
        X = M (:, 1);
        Y = M (:, 2);
        Z = M (:, 3);

        %Calcolo delle medie di X, Y, Z
        mx = mean (X);
        my = mean (Y);
        mz = mean (Z);
    
        %Calcolo delle deviazioni standard di X, Y, Z
        devX = std (X);
        devY = std (Y);
        devZ = std (Z);
    
        %Calcolo dell' errore standard
        errX = devX / sqrt (i);
        errY = devY / sqrt (i);
        errZ = devZ / sqrt (i);
    
        %Metto la riga di risultato in una matrice OUT
        OUT (i, 1:11) = [quota; ang; mx; my; mz; devX; devY; devZ; errX; errY; errZ];
        D (k, 1:3) = [mx; my; mz];
        k=k+1;
        ang = ang+10;  
        
        %Supponendo che nome = MEAS_000M1_OK.txt, calcolo il nuovo nome
        if (i<=9)
            newn = [nome(1:7) num2str(ang/10) mag(1:2) '_OK.txt']; 
        else
            newn = [nome(1:6) num2str(ang/10) mag(1:2) '_OK.txt'];
        end
    end
    filename=[car 'quota' num2str(quota) '.txt'];
    dlmwrite(filename, OUT,'newline', 'pc', 'precision', 6)
    quota=quota+1;
    fclose('all');
end    

filename2=[car 'scan' num2str(quota-quotain) '.txt'];

%%%% Passo dalla matrice D a quella F, che ha prima i 15, poi i 14 cm ecc ecc %%%%

d = size(D);
nrighe = d(1);
ncol = d(2);
f = zeros (nrighe, ncol);

nblocchi = nq;
nelem = nf;
fine = nelem*nblocchi;
fine1=fine-nelem+1;
fine2=fine1+nelem-1;
init=1;
fine3=nelem;

for i=init:nelem:nelem*nblocchi+1-nelem
    f(i:fine3, :) = D(fine1:fine2, :);
    fine1=fine1-nelem;
    fine2=fine1+nelem-1;
    fine3=fine3+nelem;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
D
f
dlmwrite(filename2, f,'newline', 'pc', 'precision', 6);
fclose('all');