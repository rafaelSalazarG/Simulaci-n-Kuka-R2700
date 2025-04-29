function main3
clear, clc, close all

puntos=100;
%Instanciamos el objeto SerialLink mediante la funcion robot y lo llamamos
%R
"""--------------------------Definicion inicial-----------------------""";
R=robot();
contador = 0;
R_MAX = 2100;
R_MIN = 1200;
ALTURA = 200;
alpha = 0;
%Definimos mediante matrices de transformación homogénea los puntos en el
%espacio cartesiano, los cuales tenemos que interpolar para describir la
%trayectoria de nuestro robot

Q_pos = zeros(1,6);
Q_vel = zeros(1,6);
Q_ace = zeros(1,6);

"""-------------------------Bucle de Trayectoria-----------------------""";

for i = -6:6
    """Trayectoria uno"""; 
    %Posicionamiento. Comenzamos en P(x1,y1,ALTURA) y
    %terminamos en P(x1,y1,0)
    q = [0+(i*30*pi/180) pi/2 -pi/2 0 -pi/8 0]; %q1 se actualiza con cada for

    %DETERMINAMOS EN QUÉ PUNTOS ESTAMOS 
    x1 = R_MAX*cos(q(1));
    y1 = R_MAX*sin(q(1));

    %Correccion del angulo de giro yaw
    yaw_add = atan2(y1,x1)*(180/pi);

    %Matrices T
    T1 = transl(x1,y1,ALTURA)*rpy2tr(0,60-180,180+yaw_add,'deg'); %Matriz inicial
    T2 = transl(x1,y1,0)*rpy2tr(0,60-180,180+yaw_add,'deg'); %Matriz final

    %ctraj para movimientos rectilineos
    [q_pos_cart,q_vel_cart,q_ace_cart] = interpolacionCartesiana(T1,T2,R,q,puntos);

    %Posicion, velocidad y aceleracion del movimiento para grafica
    Q_pos = vertcat(Q_pos,q_pos_cart);
    Q_vel = vertcat(Q_vel,q_vel_cart);
    Q_ace = vertcat(Q_ace,q_ace_cart);
    
   
    """Trayectoria dos"""; 
    %Barrido
    x2 = R_MIN*cos(Q_pos(end,1));
    y2 = R_MIN*sin(Q_pos(end,1));

    %Matrices T
    T2 = transl(x1,y1,0)*rpy2tr(0,60-180,180+yaw_add,'deg'); %Matriz inicial
    T3 = transl(x2,y2,0)*rpy2tr(0,45-180,180+yaw_add,'deg'); %Matriz final

    %ctraj para movimientos rectilineos
    [q_pos_cart,q_vel_cart,q_ace_cart] = interpolacionCartesiana(T2,T3,R,Q_pos(end,:),puntos);

    %Posicion, velocidad y aceleracion del movimiento para grafica
    Q_pos = vertcat(Q_pos,q_pos_cart);
    Q_vel = vertcat(Q_vel,q_vel_cart);
    Q_ace = vertcat(Q_ace,q_ace_cart);
    
    """Trayectoria tres"""; 
    %Levantamiento
    T3 = transl(x2,y2,0)*rpy2tr(0,45-180,180+yaw_add,'deg');
    T4 = transl(x2,y2,ALTURA)*rpy2tr(0,45-180,180+yaw_add,'deg');

    %ctraj para movimientos rectilineos
    [q_pos_cart,q_vel_cart,q_ace_cart] = interpolacionCartesiana(T3,T4,R,Q_pos(end,:),puntos);
    Q_pos = vertcat(Q_pos,q_pos_cart);
    Q_vel = vertcat(Q_vel,q_vel_cart);
    Q_ace = vertcat(Q_ace,q_ace_cart);

    if(i<6)
        """Trayectoria cuatro"""; 
        %Reposicionamiento
        alpha = ((i+1)*30*pi/180); %Para hacer girar el robot
        x3 = R_MAX*cos(alpha);
        y3 = R_MAX*sin(alpha);
        T4 = transl(x2,y2,ALTURA)*rpy2tr(0,45-180,180+yaw_add,'deg');

        q = [0+((i+1)*30*pi/180) pi/2 -pi/2 0 -pi/8 0];
        x1 = R_MAX*cos(q(1));
        y1 = R_MAX*sin(q(1));
        yaw_add = atan2(y1,x1)*(180/pi);
    
        T5 = transl(x3,y3,ALTURA)*rpy2tr(0,60-180,180+yaw_add,'deg');
        q4 = q_pos_cart(end,:);
        q5 = pieper_geom(R,T5,q4,true);

        %ctraj para movimientos rectilineos
        [q_pos_cart,q_vel_cart,q_ace_cart] = interpolacionArticular(q4,q5,puntos);
        Q_pos = vertcat(Q_pos,q_pos_cart);
        Q_vel = vertcat(Q_vel,q_vel_cart);
        Q_ace = vertcat(Q_ace,q_ace_cart);
    end
end

"""--------------------------Grafica y Animacion-----------------------""";

Q_pos(1,:) = Q_pos(2,:);
%Graficas de posicion, velocidad y aceleracion
hold on
figure(1)
subplot(3,1,1)
title("Desplazamiento interp. cartesiana")
qplot(Q_pos)
subplot(3,1,2)
title("Velocidad interp. cartesiana")
qplot(Q_vel)
subplot(3,1,3)
title("Aceleracion interp. cartesiana")
qplot(Q_ace)
%Animacion de trayectoria
figure(2)
R.plot3d(Q_pos,'path','modelo3Defector','scale',0.001,'notiles','nowrist','trail',{'r', 'LineWidth', 2},'fps',120,'view',[160 7]);
view(3)


"""--------------------Funciones de interpolacion-------------------""";
end

function [q_pos_art,q_vel_art,q_ace_art] = interpolacionArticular(q1,q2,puntos)
    %jtraj() para trayectorias curvilineas
    [q_pos_art,q_vel_art,q_ace_art] = jtraj(q1,q2,puntos);
end

function [q_pos_cart,q_vel_cart,q_ace_cart] = interpolacionCartesiana(Ti,Tf,R,qPrevia,puntos)
    %Interpolamos en el espacio cartesiano
    Tcartesiana=ctraj(Ti,Tf,puntos);
    %Realizamos la cinemática inversa entre cada trayectoria
    q_pos_cart=zeros(puntos,6);
    qAnterior=qPrevia;
    for i=1:puntos
        q_pos_cart(i,:) = pieper_geom(R,Tcartesiana(:,:,i),qAnterior,true);
        qAnterior=q_pos_cart(i,:);
        q_vel_cart=diff(q_pos_cart)/(1/puntos);
        q_ace_cart=diff(q_vel_cart)/(1/puntos);
    end    
end

