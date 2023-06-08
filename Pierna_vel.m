%Limpieza de pantalla
clear
close all
clc

%Declaración de variables simbólicas
syms q9(t) q8(t) q7(t) q6(t) q5(t) q4(t) q3(t) q2(t) q1(t) t a1 a2 a3 a4 a5 a6 a7 a8 a9 x y z v1 v2 v3 v4 v5 v6 v7 v8 v9

%Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP=[0 0 0 0 0 0 0 0 0];

%Creamos el vector de coordenadas articulares
Q= [q1 q2 q3 q4 q5 q6 q7 q8 q9];
%disp('Coordenadas generalizadas');
%pretty (Q);
%Creamos el v
% ector de velocidades generalizadas
Qp= diff(Q, t);
%Qp = [v1 v2 v3 v4 v5 v6 v7 v8 v9];

%disp('Velocidades generalizadas');
%pretty (Qp);
%Número de grado de libertad del robot
GDL= size(RP,2);
GDL_str= num2str(GDL);
 
 
%{
%% pos 1 
%Posición de la articulación 
P(:,:,1)= [0;0;0];
%Matriz de rotación de la junta
R(:,:,1)= [1 0 0;
           0 1 0;
           0 0 1];
%}
%% pos  2
%Posición de la articulación 
P(:,:,1)= [x;y;z];
%Matriz de rotación de la junta 
R(:,:,1)=[1       0           0;
           0    cos(-q2)    -sin(-q2);
           0    sin(-q2)    cos(-q2)];

%Posición de la articulación 
P(:,:,2)= [0;0;0];
%Matriz de rotación de la junta 
R(:,:,2)= [cos(-q1) 0 sin(-q1);
           0 1 0 ;
           -sin(-q1) 0 cos(-q1)];

%% pos 3
%Posición de la articulación
P(:,:,3)= [0;0;0];
%Matriz de rotación de la junta
R(:,:,3)= [1 0 0;
           0 cos(-q3) -sin(-q3);
           0 sin(-q3) cos(-q3)];

%Posición de la articulación 
P(:,:,4)= [0;0;0];
%Matriz de rotación de la junta 
R(:,:,4)= [cos(-q2) 0 sin(-q2);
            0 1 0;
           -sin(-q2) 0 cos(-q2)];

%% pos 4
P(:,:,5)=[a1;0;0];
R(:,:,5)=[1 0 0;
          0 1 0;
          0 0 1];
%% pos 5
%Posición de la articulación
P(:,:,6)= [a2;0;0];
%Matriz de rotación de la junta
R(:,:,6)= [cos(q4) -sin(q4) 0;
           sin(q4) cos(q4)  0
            0 0 1];
%% pos 6
%Posición de la articulación 
P(:,:,7)= [0;0;0];
%Matriz de rotación de la junta 
R(:,:,7)= [cos(-q1) 0 sin(-q1);
            0 1 0;
           -sin(-q1) 0 cos(-q1)];
%Posición de la articulación
P(:,:,8)= [0;0;0];
%Matriz de rotación de la junta
R(:,:,8)= [cos(q6) -sin(q6) 0;
           sin(q6) cos(q6)  0
            0 0 1];

%% pos 7
P(:,:,9)=[0;0;a3];
R(:,:,9)=[1 0 0;
          0 1 0;
          0 0 1];


%Creamos un vector de ceros
Vector_Zeros= zeros(1, 3);

%Inicializamos las matrices de transformación Homogénea locales
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las matrices de transformación Homogénea globales
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
%Inicializamos las posiciones vistas desde el marco de referencia inercial
PO(:,:,GDL)= P(:,:,GDL); 
%Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,GDL)= R(:,:,GDL); 


for i = 1:GDL
    i_str= num2str(i);
   %disp(strcat('Matriz de Transformación local A', i_str));
    A(:,:,i)=simplify([R(:,:,i) P(:,:,i); Vector_Zeros 1]);
   %pretty (A(:,:,i));

   %Globales
    try
       T(:,:,i)= T(:,:,i-1)*A(:,:,i);
    catch
       T(:,:,i)= A(:,:,i);
    end
    disp(strcat('Matriz de Transformación global T', i_str));
    T(:,:,i)= simplify(T(:,:,i));
    pretty(T(:,:,i))

    RO(:,:,i)= T(1:3,1:3,i);
    PO(:,:,i)= T(1:3,4,i);
    %pretty(RO(:,:,i));
    %pretty(PO(:,:,i));
end

%{
%Calculamos el jacobiano lineal de forma diferencial
%disp('Jacobiano lineal obtenido de forma diferencial');
%Derivadas parciales de x respecto a th1 y th2
Jv11= functionalDerivative(PO(1,1,GDL), q2);
Jv12= functionalDerivative(PO(1,1,GDL), q1);
Jv13= functionalDerivative(PO(1,1,GDL), q3);
Jv14= functionalDerivative(PO(1,1,GDL), q2);
Jv15= functionalDerivative(PO(1,1,GDL), a1);
Jv16= functionalDerivative(PO(1,1,GDL), q4);
Jv17= functionalDerivative(PO(1,1,GDL), q1);
Jv18= functionalDerivative(PO(1,1,GDL), q6);
Jv19= functionalDerivative(PO(1,1,GDL), a3);
%Derivadas parciales de y respecto a th1 y th2
Jv21= functionalDerivative(PO(2,1,GDL), q2);
Jv22= functionalDerivative(PO(2,1,GDL), q1);
Jv23= functionalDerivative(PO(2,1,GDL), q3);
Jv24= functionalDerivative(PO(2,1,GDL), q2);
Jv25= functionalDerivative(PO(2,1,GDL), a1);
Jv26= functionalDerivative(PO(2,1,GDL), q4);
Jv27= functionalDerivative(PO(2,1,GDL), q1);
Jv28= functionalDerivative(PO(2,1,GDL), q6);
Jv29= functionalDerivative(PO(2,1,GDL), a3);
%Derivadas parciales de z respecto a th1 y th2
Jv31= functionalDerivative(PO(3,1,GDL), q2);
Jv32= functionalDerivative(PO(3,1,GDL), q1);
Jv33= functionalDerivative(PO(3,1,GDL), q3);
Jv34= functionalDerivative(PO(3,1,GDL), q2);
Jv35= functionalDerivative(PO(3,1,GDL), a1);
Jv36= functionalDerivative(PO(3,1,GDL), q4);
Jv37= functionalDerivative(PO(3,1,GDL), q1);
Jv38= functionalDerivative(PO(3,1,GDL), q6);
Jv39= functionalDerivative(PO(3,1,GDL), a3);


%Creamos la matríz del Jacobiano lineal
jv_d=simplify([Jv11 Jv12 Jv13;
              Jv21 Jv22 Jv23;
              Jv31 Jv32 Jv33]);
%pretty(jv_d);
%}

%Calculamos el jacobiano lineal de forma analítica
Jv_a(:,GDL)=PO(:,:,GDL);
Jw_a(:,GDL)=PO(:,:,GDL);

for k= 1:GDL
    if RP(k)==0 
       %Para las juntas de revolución
        try
            Jv_a(:,k)= cross(RO(:,3,k-1), PO(:,:,GDL)-PO(:,:,k-1));
            Jw_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)= cross([0,0,1], PO(:,:,GDL));%Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa tambien será 0
            Jw_a(:,k)=[0,0,1];%Si no hay matriz de rotación previa se obtiene la Matriz identidad
         end
     else
%         %Para las juntas prismáticas
        try
            Jv_a(:,k)= RO(:,3,k-1);
        catch
            Jv_a(:,k)=[0,0,1];
        end
            Jw_a(:,k)=[0,0,0];
     end
 end    

Jv_a= simplify (Jv_a);
Jw_a= simplify (Jw_a);
%disp('Jacobiano lineal obtenido de forma analítica');
%pretty (Jv_a);
%disp('Jacobiano ángular obtenido de forma analítica');
%pretty (Jw_a);


disp('Velocidad lineal obtenida mediante el Jacobiano lineal');
V=simplify (Jv_a*Qp');
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano angular');
W=simplify (Jw_a*Qp');
    pretty(W);