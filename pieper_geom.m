function Q = pieper_geom(R,T,q0,mejor)
    
    %Limpiamos offset para generalizar el calculo
    offsets = R.offset;
    R.offset = zeros(1,6);
    
    flag=[1,1,1,1,1,1,1,1];%ponemos en 0 cuando las soluciones no son posibles
    limout=[1,1,1,1,1,1,1,1];%0 si esta fuera de los limites
    
    %Eslabones
    
    L1 = R.links(1).d;
    L2 = R.links(1).a;
    L3 = R.links(2).a;
    L4 = R.links(3).a;
    L5 = R.links(4).d;
    L12 = sqrt(L1^2+L2^2);
    L45 = sqrt(L4^2+L5^2);


    %Centro muñeca:
    % T = T.double;
    %Desacoplamos la matriz de base y tool y tenemos T06
    T = R.base.double\T/R.tool.double; %La desacoplo de tool y base
    PC = T(1:3,4)-R.links(6).d*T(1:3,3); %centro de la muñeca
    
    %Verificamos que no este fuera del rango
    %de los primeros 3gdl
    R1 = sqrt(PC(1)^2+PC(2)^2);
    R2 = sqrt(PC(1)^2+PC(2)^2+ PC(3)^2);
    if R2 > (L12+L3+L45)
        disp('Posicion fuera de rango');
        flag(:) = 0;
    else
        %Matriz de vectores solucion
        q = zeros(4,3);
        %Nuestro Q1 tiene dos posibles valores
    
        for i = 1:2
            q(i,1) = atan2(PC(2),PC(1));
        end
        for i =3:4
            if q(1,1) > 0
                q(i,1) = atan2(PC(2),PC(1)) - pi;
            else
                q(i,1) = atan2(PC(2),PC(1)) + pi;
            end
        end
    
    
        %Buscamos Q2
        PCdh = ones(4,1);
        PCdh(1:3,1) = PC;
        r = zeros(4,1);
        beta = zeros(4,1);
        alpha = zeros(4,1);
    
        %Q2 tiene 2 valores para cada valor de Q1
        for i = 1:4
            T1 = R.links(1).A(q(i,1)).double;%esto es T10
            PC1 = T1\PCdh;%PC respecto del sist 1
            r(i) = sqrt(PC1(1)^2+PC1(2)^2);
            beta(i) = atan2(PC1(2),PC1(1));
            alpha(i) = acos((L45^2-L3^2-r(i)^2)/(-2*L3*r(i)));
            %Por si beta nos da valores imaginarios
            if  ~isreal(alpha(i))
                disp('Alpha da numeros img');
                flag(i)=0;
            end
            %Guardamos los valores de Q2
            if i == 1 || i == 3
                q(i,2) = beta(i,1) + real(alpha(i,1));
            else
                q(i,2) = beta(i,1) - real(alpha(i,1));
            end

        end
        
        gamma = zeros(4,1);
        delta = zeros(4,1);

        %Buscamos Q3
        %Q3 tiene un valor para cada Q2
        for i = 1:4
            T1 = R.links(1).A(q(i,1)).double;
            T2 = T1 * R.links(2).A(q(i,2)).double;%esto es T20
            PC2 = T2\PCdh;%PC respecto del sist 2
            %Guardamos Q3
            gamma(i,3) = atan2(L5,L4);
            delta(i,3) = atan2(PC2(2),PC2(1));
            q(i,3) = gamma(i,3) + delta(i,3);
            % if q(i,3) > 0
            %     q(i,3) = q(i,3) - pi/2;
            % elseif q(i,3) < 0
            %     q(i,3) = q(i,3) + pi/2;
            % end
        end
    
    end
    
    %Comenzamos el calculo de q4, q5 y q6, y tenemos en cuenta que
    %para cada solucion de la posicion de la muñeca, tenemos 2
    %posibles q4, es decir, pasamos a tener 8 posibles soluciones
    %para una matriz T dato que nos indique la posicion y la
    %orientacion del efector final
    
    qq = zeros(8,3);
    
    qq(1,:) = q(1,:);
    qq(2,:) = q(1,:);
    qq(3,:) = q(2,:);
    qq(4,:) = q(2,:);
    qq(5,:) = q(3,:);
    qq(6,:) = q(3,:);
    qq(7,:) = q(4,:);
    qq(8,:) = q(4,:);
    
    %Calculo T36 y q4
    
    
    %Va de 2 en 2 para no repetir soluciones
    for i = 1:2:7
        T1 = R.links(1).A(qq(i,1)).double;
        T2 = T1 * R.links(2).A(qq(i,2)).double;
        T3 = T2 * R.links(3).A(qq(i,3)).double;%esto es T30
        T63 = T3\T; %Obtenemos T63
        %Guardamos Q4
        qq(i,4) = atan2(T63(2,3), T63(1,3)); %CREO QUE SE CALCULA EL ÁNGULO OPUESTO AL DE LA PROYECCIÓN
        if qq(i,4) > 0
            qq(i+1,4) = qq(i,4) - pi;
        else
            qq(i+1,4) = qq(i,4) + pi;
        end
    end
    %Calculamos q5 para cada q4 y lo guardamos
    for i = 1:8
        T1 = R.links(1).A(qq(i,1)).double;
        T2 = T1 * R.links(2).A(qq(i,2)).double;
        T3 = T2 * R.links(3).A(qq(i,3)).double;%esto es T30
        T63 = T3\T; %Obtenemos T63
        T43 = R.links(4).A(qq(i,4)).double;
        T64 = T43\T63;
        qq(i,5) = atan2(T64(2,3), T64(1,3)) + pi/2;
    end
    %Calculamos q6 y lo guardamos
    for i = 1:8
        T1 = R.links(1).A(qq(i,1)).double;
        T2 = T1 * R.links(2).A(qq(i,2)).double;
        T3 = T2 * R.links(3).A(qq(i,3)).double;%esto es T30
        T63 = T3\T; %Obtenemos T63
        T43 = R.links(4).A(qq(i,4)).double;
        T64 = T43\T63;
        T54 = R.links(5).A(qq(i,5)).double;
        T65 = T54\T64;
        qq(i,6) = atan2(T65(2,1), T65(1,1));
    end
    
    R.offset = offsets;
    qq = qq - R.offset;
    
    for i=1:8
        for j=1:6
            if qq(i,j)<R.qlim(j,1)
                limout(i)=0;
            elseif qq(i,j)>R.qlim(j,2)
                limout(i)=0;
            end
        end
    end

    sol=0;
    min=10000;
    if mejor %si es true
        qaux = qq - q0;
        normas = zeros(8,1);
        for i=1:8
            normas(i) = norm(qaux(i,:));
            if flag(i) == 1 && limout(i) == 1 && normas(i)<min
                min = normas(i);
                sol = i;
            end
        end
    
        if sol == 0
            disp('No hay solucion');
            Q = qq(1,:);
        else
            Q = qq(sol,:);
        end
    else
        Q = qq;
        sol_posibles = find(flag == 1 & limout == 1);
        if sol_posibles ~= 0
            fprintf('las soluciones posibles son: ')
            disp(sol_posibles);
        end
        for i=1:8
            if flag(i) == 1 && limout(i) == 1
                disp(Q(i,:)*180/pi);
            end
        end
    end
end


