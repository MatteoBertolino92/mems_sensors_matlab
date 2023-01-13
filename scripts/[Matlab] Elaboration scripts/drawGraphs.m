function [u, v, w, uq, vq, wq, um, vm, wm, cucu, cucv, cucz, m3d, modm, uMap, vMap, wMap, uxgq, uygq] = disegnaGrafici (ndscan, hscan, i, thetaq, x, y, z, scalaq, scala, x0, y0, z0, nth)
    [nome, path]=uigetfile('*.txt', ['Open file called *scan*.txt per il sensore ', num2str(i)]);
    fp = fopen([path, nome], 'r');
    M  = fscanf(fp, '%g %*c %g %*c %g', [3, inf]);
    M = M';
    fclose(fp);

    u = M(:, 1); %In w0, u0 e v0 ci sono rispettivamente le 396 medie di x, y, z. 
    v = M(:, 2); %Ciascun valore è la media su 50 valori di una misurazione, ad altezza e angolo fissate.
    w = M(:, 3); %Ogni 36 righe gli angoli si ripetono..! Sono vettori colonna di 396 elementi = 36*11.
    
    u1=u-mean(u);
    v1=v-mean(v);
    w1=w-mean(w);
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
    
   
    figure('Name', ' Componenti quiver')
    hold on
    grid on 
    quiver3 (x(1), y(1), z(1), 0, 0, z(ndscan*hscan)-z(1), 'k')
    hold on
    grid on
    quiver3 (x, y, z, uq-um, vq-vm, wq-wm, scalaq) %Rimuovo la componente continua 
    title(['Componenti per sensore ', num2str(i)])
    axis equal
    axis square

%     figure
%     uq=uq-uq;
%     quiver3 (x, y, z, uq, vq-vm, wq-wm, scalaq) %Rimuovo la componente continua
%     title(['Radiali e tangenziali su piano orizzontale per sensore ', num2str(i)])
%     axis equal
%     axis square

    %La scatter disegna dei pallini pieni, con il modulo colorato
    m3d=sqrt(u.^2+v.^2+w.^2);
    figure('Name', ' Scatter modulo')
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
    
    u=u1;
    v=v1;
    w=w1;
    
    uMap = reshape (w', nth, hscan)'; uMap(:, nth+1) = uMap(:, 1); %componente radiale per la mappa 
    vMap = reshape (v', nth, hscan)'; vMap(:, nth+1) = vMap(:, 1); %componente tangenziale per la mappa 
    wMap = reshape (u', nth, hscan)'; wMap(:, nth+1) = wMap(:, 1); %componente verticale per la mappa

    figure('Name', ' Modulo')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, modm, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Modulo per sensore ', num2str(i)])

    figure('Name', ' Radiale')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, uMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Radiale per sensore ', num2str(i)])

    figure('Name', ' Tangenziale')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, vMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Tangenziale per sensore ', num2str(i)])

    figure('Name', ' Verticale')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, wMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title(['Verticale per sensore ', num2str(i)])
 
%radiali su piano orizzontale
     figure('Name', ' Radiali su piano orizzontale');
     uq1=uq-um;
     uxg=uq1.*cos(thetaq); uyg=uq1.*sin(thetaq);
     uxgq=reshape(uxg, nth, hscan)';    uxgq(:,nth+1) = uxgq(:,1);
     uygq=reshape(uyg, nth, hscan)';    uygq(:,nth+1) = uygq(:,1);

     for j=1:hscan
     %Prima freccia rossa
     subplot(4,4,j);
     quiver(cucu(j, 1:2), cucv(j, 1:2), uxgq(j, 1:2), uygq(j, 1:2), 'r', 'Linewidth', 1.25);
     hold on;
     subplot (4,4,j);
     quiver(cucu(j, 2:ndscan), cucv(j, 2:ndscan), uxgq(j, 2:ndscan), uygq(j, 2:ndscan), 'b', 'Linewidth', 1.3);
     hold on
     subplot(4,4,j); plot(x0,y0,'k','LineWidth', 1);
     axis equal
     grid on
     title(num2str(19-j));
     end 
    
      
%tangenziali su piano orizzontale
figure('Name', ' Tangenziali su piano orizzontale')
     vq1=vq-vm; vq1=vq-vm;
     vxg=vq1.*cos(thetaq); vyg=vq1.*sin(thetaq);
     vxgq=reshape(vxg, nth, hscan)';    vxgq(:,nth+1) = vxgq(:,1);
     vygq=reshape(vyg, nth, hscan)';    vygq(:,nth+1) = vygq(:,1);

     for j=1:hscan
     %Prima freccia rossa
     subplot(4,4,j);
     quiver(cucu(j, 1:2), cucv(j, 1:2), vxgq(j, 1:2), vygq(j, 1:2), 'r', 'Linewidth', 1.25);
     hold on;
     subplot (4,4,j);
     quiver(cucu(j, 2:ndscan), cucv(j, 2:ndscan), vxgq(j, 2:ndscan), vygq(j, 2:ndscan), 'b', 'Linewidth', 1.3);
     hold on
     subplot(4,4,j); plot(x0,y0,'k','LineWidth', 1);
     axis equal
     grid on
     title(num2str(19-j));
     end 
      
     axgq=uxgq+vxgq;
     bygq=uygq+vygq;
     
 figure('Name', ' Tangenziali e radiali su piano orizzontale')
      for j=1:hscan
      %Prima freccia rossa
      subplot(4,4,j);
      quiver(cucu(j, 1:2), cucv(j, 1:2), axgq(j, 1:2), uygq(j, 1:2) + bygq(j, 1:2), 'r', 'Linewidth', 1.25);
      hold on;
      subplot (4,4,j);
      quiver(cucu(j, 2:ndscan), cucv(j, 2:ndscan), axgq(j, 2:ndscan), bygq(j, 2:ndscan), 'b', 'Linewidth', 1.3);
      hold on
      subplot(4,4,j); plot(x0,y0,'k','LineWidth', 1);
      axis equal
      grid on
      title(num2str(19-j));
      end 
end