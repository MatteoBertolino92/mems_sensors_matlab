function calcoloGradiente(uq, vq, wq, x, y, z, thetaq, scalaq, u, v, w, x0, y0, z0, hscan, ndscan, nth, m3d, uxgq, uygq)
    figure('Name', ' Quiver gradiente')
    %hold on
    %quiver3 (x(1), y(1), z(1), 0, 0, z(ndscan*hscan)-z(1), 'k')

    quiver3 (x, y, z, uq, vq, wq, scalaq) %Rimuovo la componente continua
    title('Componenti Gradiente')
    grid on
    axis equal
    axis square

    cucu = repmat(x0, hscan, 1);     cucu(:,nth+1)=cucu(:,1);
    cucv = repmat(y0, hscan, 1);     cucv(:,nth+1)=cucv(:,1);
    cucz = repmat(z0', 1, nth);      cucz(:,nth+1)=cucz(:,1);
    modm = reshape(m3d',nth,hscan)'; modm(:, nth+1) = modm (:, 1); %la reshape incolonna la riga, per cui devo avere un vettore riga...
    uMap = reshape (w', nth, hscan)'; uMap(:, nth+1) = uMap(:, 1); %componente radiale per la mappa 
    vMap = reshape (v', nth, hscan)'; vMap(:, nth+1) = vMap(:, 1); %componente tangenziale per la mappa 
    wMap = reshape (u', nth, hscan)'; wMap(:, nth+1) = wMap(:, 1); %componente verticale per la mappa

    figure('Name', ' Modulo gradiente')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, modm, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title('Modulo Gradiente')
    
    figure('Name', ' Radiale gradiente')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, uMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title('Radiale Gradiente')

    figure('Name', ' Tangenziale gradiente')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, vMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title('Tangenziale Gradiente')

    figure('Name', ' Verticale gradiente')
    hold on
    grid on
    plot3(x(1),y(1),z(1), 'ro', 'Linewidth', 4)
    surf (cucu, cucv, cucz, wMap, 'FaceColor','interp', 'EdgeColor','none', 'FaceLighting', 'phong');
    axis equal
    axis square
    title('Verticale Gradiente')
    uxgq=uxgq-uxgq;

%radiali su piano orizzontale
     figure('Name', ' Radiali gradiente su piano orizzontale');
     uxg=uq.*cos(thetaq); uyg=uq.*sin(thetaq);
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
figure('Name', ' Tangenziali gradiente su piano orizzontale');
     vxg=vq.*cos(thetaq); vyg=vq.*sin(thetaq);
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
     
 figure('Name', ' Tangenziali e radiali gradiente su piano orizzontale')
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