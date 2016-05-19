clear all
close all
clc

numeroFile = input ('Quanti file vuoi fare? ');
numeroElem = input ('Quante misurazioni per file devo acquisire? ');
numeroElem = numeroElem*2;
serialPort = input ('A quale porta seriale sei connesso? (es. COM13) ', 's');
nomeStartA = 'MEAS_00';
nomeStartB = 'MEAS_0';


s = serial(serialPort);
set(s, 'BaudRate', 230400, 'Databits', 8, 'Parity', 'none', 'StopBits', 1, 'FlowControl', 'none');
fopen(s);


for i=1:20 %prime 20 misurazioni a vuoto
    string = fscanf(s);
end

for i=1:numeroFile
    %Calcolo nome file
    num=i-1;
    if i<=10
        nomeEff = strcat(nomeStartA, num2str(num), '.txt');
    else
        nomeEff = strcat (nomeStartB, num2str(num), '.txt');
    end
    fp = fopen (nomeEff, 'w');
    for j=1:numeroElem
        string=fscanf(s);
        fprintf(fp, string);
    end
    fclose (fp);
    fprintf ('File %d acquisito. Premere un tasto per continuare\n', i);
    pause;
end
    
    fprintf('Fine\n');
    fclose(s);
    
choiche = input('Vuoi splittare i risultati in 2 parti, ciascuno con i dati M1 e M2 (si/no)? ', 's');
if choiche=='si'
    mkdir ('M1split'); mkdir ('M2split');
    for i=1:numeroFile
        num=i-1;
        if i<=10
            nomeDaUsare = strcat (nomeStartA, num2str(num), '.txt')
            nomeEffM1 = strcat(nomeStartA, num2str(num), 'M1.txt');
            nomeEffM2 = strcat(nomeStartA, num2str(num), 'M2.txt');
        else
            nomeDaUsare = strcat (nomeStartB, num2str(num), '.txt');
            nomeEffM1 = strcat(nomeStartB, num2str(num), 'M1.txt');
            nomeEffM2 = strcat(nomeStartB, num2str(num), 'M2.txt');
        end
        fp = fopen (nomeDaUsare, 'r');
        fp1= fopen (nomeEffM1, 'w');
        fp2= fopen (nomeEffM2, 'w');
        M = fscanf (fp, '%f %c %c %c %f %f %f %f %c %c %c %f %f %f', [14, numeroElem]);
        M = M';
        
        for j=1:numeroElem
            if M(j, 2)== 77
                fprintf (fp1, '%f %f %f\n', M(j, 5), M(j, 6), M(j, 7));
                fprintf (fp2, '%f %f %f\n', M(j, 12), M(j, 13), M(j, 14));
            end
        end
        fclose (fp);
        fclose(fp1);
        fclose(fp2);
        copyfile(nomeEffM1, 'M1split'); copyfile(nomeEffM2, 'M2split'); 
        %delete(nomeEffM1); delete(nomeEffM2);
    end   
end