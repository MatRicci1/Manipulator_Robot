function c=ComputeCCTorques(dq_dt,q)
%ComputeCCTorques
% Calculates the term associated to the Coriolis and Centrigual Forces
% Returns a vector length(q)x1, C*dq_dt

    % Initialisation
    c=zeros(length(q),1)';
    iI = zeros(3,3,length(q));

    % Parametres
    m=[15 10 1 7 1 0.5]';
    iI(:,:,1)=[0.80 0 0.05 ; 0 0.80 0 ; 0.05  0  0.10];
    iI(:,:,2)=[0.10  0  0.10; 0  1.50 0 ; 0.10  0  1.50];
    iI(:,:,3)=[0.05 0 0 ; 0 0.01 0 ; 0 0 0.05];
    iI(:,:,4)=[0.50 0 0 ; 0 0.50 0 ; 0 0 0.05];
    iI(:,:,5)=[0.01 0 0 ; 0 0.01 0 ; 0 0 0.01];
    iI(:,:,6)=[0.01 0 0 ; 0 0.01 0 ; 0 0 0.01];
    Jm=[10*10^(-6) 10*10^(-6) 10*10^(-6) 10*10^(-6) 10*10^(-6) 10*10^(-6)]';
    x_G1 = 0; y_G1 = 0; z_G1 = -0.25;
    x_G2 = 0.35; y_G2 = 0; z_G2 = 0;
    x_G3 = 0; y_G3 = -0.1; z_G3 = 0;
    x_G4 = 0; y_G4 = 0; z_G4 = 0;
    x_G5 = 0; y_G5 = 0; z_G5 = 0;
    x_G6 = 0; y_G6 = 0; z_G6 = 0;
    x_G=[x_G1 x_G2 x_G3 x_G4 x_G5 x_G6]';
    y_G=[y_G1 y_G2 y_G3 y_G4 y_G5 y_G6]';
    z_G=[z_G1 z_G2 z_G3 z_G4 z_G5 z_G6]';
    d3 = 0.7; 
    r1 = 0.5; r3=0.0 ; r4 = 0.2;
    alpha = [ 0 , pi/2 , 0 , pi/2 , -pi/2 , pi/2 ] ;
    alpha = [ 0 , pi/2 , 0 , pi/2 , -pi/2 , pi/2 ] ;
    Offset = [ 0 , 0 , pi/2 , 0 , 0 , 0  ]' ;
    theta = q + Offset;


    % Parametres relatifs aux corps
    % Axe 1 %[0.80 0 0.05 ; 0 0.80 0 ; 0.05  0  0.10]
    XX1=0.80;
    XY1=0;
    XZ1=0.05;
    YY1=0.80;
    YZ1=0;
    ZZ1=0.10;

    M1=15;

    MX1=0*M1; %x_G1 = 0; y_G1 = 0; z_G1 = -0.25;
    MY1=0*M1;
    MZ1=-0.25*M1;

    % Axe 2 %[0.10  0  0.10; 0  1.50 0 ; 0.10  0  1.50]
    XX2=0.10;
    XY2=0;
    XZ2=0.10;
    YY2=1.50;
    YZ2=0;
    ZZ2=1.50;

    M2=10;

    MX2=0.35*M2; %x_G2 = 0.35; y_G2 = 0; z_G2 = 0;
    MY2=0*M2;
    MZ2=0*M2;

    % Axe 3 %[0.05 0 0 ; 0 0.01 0 ; 0 0 0.05]
    XX3=0.05;
    XY3=0;
    XZ3=0;
    YY3=0.01;
    YZ3=0;
    ZZ3=0.05;

    M3=1;

    MX3=0*M3; %x_G3 = 0; y_G3 = -0.1; z_G3 = 0;
    MY3=-0.1*M3;
    MZ3=0*M3;

    % Axe 4 %[0.50 0 0 ; 0 0.50 0 ; 0 0 0.05]
    XX4=0.50;
    XY4=0;
    XZ4=0;
    YY4=0.50;
    YZ4=0;
    ZZ4=0.05;

    M4=7;

    MX4=0*M4; %x_G4 = 0; y_G4 = 0; z_G4 = 0;
    MY4=0*M4;
    MZ4=0*M4;

    % Axe 5 %[0.01 0 0 ; 0 0.01 0 ; 0 0 0.01]
    XX5=0.01;
    XY5=0;
    XZ5=0;
    YY5=0.01;
    YZ5=0;
    ZZ5=0.01;

    M5=1;

    MX5=0*M5; %x_G5 = 0; y_G5 = 0; z_G5 = 0;
    MY5=0*M5; 
    MZ5=0*M5;

    % Axe 6 %[0.01 0 0 ; 0 0.01 0 ; 0 0 0.01]
    XX6=0.01;
    XY6=0;
    XZ6=0;
    YY6=0.01;
    YZ6=0;
    ZZ6=0.01;

    M6=0.5;

    MX6=0*M6; %x_G6 = 0; y_G6 = 0; z_G6 = 0;
    MY6=0*M6;
    MZ6=0*M6;

    % Valeurs angulaires
    t1=theta(1);
    t2=theta(2);
    t3=theta(3);
    t4=theta(4);
    t5=theta(5);
    t6=theta(6);

    C1 = cos(t1);
    S1 = sin(t1);
    C2 = cos(t2);
    S2 = sin(t2);
    C3 = cos(t3);
    S3 = sin(t3);
    C4 = cos(t4);
    S4 = sin(t4);
    C5 = cos(t5);
    S5 = sin(t5);
    C6 = cos(t6);
    S6 = sin(t6);

    % Vitesses articulaires
    QP1=dq_dt(1);
    QP2=dq_dt(2);
    QP3=dq_dt(3);
    QP4=dq_dt(4);
    QP5=dq_dt(5);
    QP6=dq_dt(6);
    
    % Parametres regroupes
    XXR6 = XX6 - YY6;

    MR5 = M5 + M6;
    XXR5 = XX5 - YY5 + YY6;
    ZZR5 = YY6 + ZZ5;
    MYR5 = MY5 - MZ6;

    LamMS134 = 2*r4;
    LamM14 = r4^2;
    MR4 = M4 + MR5;
    XXR4 = XX4 - YY4 + YY5;
    ZZR4 = YY5 + ZZ4;
    MYR4 = MY4 + MZ5;

    LamMS133 = 2*r3;
    LamM13 = r3^2;
    LamM33 = -d3*r3;
    LamM43 = d3^2 + r3^2;
    LamM63 = d3^2;
    MR3 = M3 + MR4;
    XXR3 = LamM14*MR4 + LamMS134*MZ4 + XX3 - YY3 + YY4;
    ZZR3 = LamM14*MR4 + LamMS134*MZ4 + YY4 + ZZ3;
    MYR3 = -MR4*r4 + MY3 - MZ4;

    YYR2 = LamM43*MR3 + LamMS133*MZ3 + YY2 + YY3;
    MZR2 = MR3*r3 + MZ2 + MZ3;
    MR2 = M2 + MR3;
    XXR2 = LamM13*MR3 + LamMS133*MZ3 + XX2 + YY3 - YYR2;
    XZR2 = LamM33*MR3 - MZ3*d3 + XZ2;
    % ZZR2 = IA2 + LamM63*MR3 + ZZ2;
    ZZR2 = LamM63*MR3 + ZZ2;
    MXR2 = MR3*d3 + MX2;

    MR1 = M1 + MR2;
    % ZZR1 = IA1 + YYR2 + ZZ1;
    ZZR1 = YYR2 + ZZ1;

    IA1=0;
    IA2=0;
    IA3=0;
    IA4=0;
    IA5=0;
    IA6=0;

    FX6=0;
    FX5=0;
    FX4=0;
    FX3=0;
    FX2=0;
    FX1=0;
    FY6=0;
    FY5=0;
    FY4=0;
    FY3=0;
    FY2=0;
    FY1=0;
    FZ6=0;
    FZ5=0;
    FZ4=0;
    FZ3=0;
    FZ2=0;
    FZ1=0;
    CX6=0;
    CX5=0;
    CX4=0;
    CX3=0;
    CX2=0;
    CX1=0;
    CY6=0;
    CY5=0;
    CY4=0;
    CY3=0;
    CY2=0;
    CY1=0;
    CZ6=0;
    CZ5=0;
    CZ4=0;
    CZ3=0;
    CZ2=0;
    CZ1=0;
    FS1=0;
    FS2=0;
    FS3=0;
    FS4=0;
    FS5=0;
    FS6=0;
    FV1=0;
    FV2=0;
    FV3=0;
    FV4=0;
    FV5=0;
    FV6=0;


    % Calcul des couples de Coriolis et centrifuge
    
    G3=0; % Pour supprimer les termes de gravite

    DV61 = QP1^2;
    W12 = QP1*S2;
    W22 = C2*QP1;
    WP12 = QP2*W22;
    WP22 = -QP2*W12;
    DV12 = W12^2;
    DV22 = W12*W22;
    DV42 = W22^2;
    DV62 = QP2^2;
    U112 = -DV42 - DV62;
    U312 = -2*WP22;
    U222 = -DV12 - DV62;
    U322 = 2*WP12;
    U332 = -DV12 - DV42;
    VP12 = G3*S2;
    VP22 = C2*G3;
    W13 = C3*W12 + S3*W22;
    W23 = C3*W22 - S3*W12;
    W33 = QP2 + QP3;
    WP13 = C3*WP12 + QP3*W23 + S3*WP22;
    WP23 = C3*WP22 - QP3*W13 - S3*WP12;
    DV13 = W13^2;
    DV23 = W13*W23;
    DV33 = W13*W33;
    DV43 = W23^2;
    DV53 = W23*W33;
    DV63 = W33^2;
    U113 = -DV43 - DV63;
    U313 = DV33 - WP23;
    U223 = -DV13 - DV63;
    U323 = DV53 + WP13;
    U133 = DV33 + WP23;
    U233 = DV53 - WP13;
    U333 = -DV13 - DV43;
    VSP13 = U112*d3 + VP12;
    VSP23 = DV22*d3 + VP22;
    VSP33 = U312*d3 + U332*r3;
    VP13 = C3*VSP13 + S3*VSP23;
    VP23 = C3*VSP23 - S3*VSP13;
    W14 = C4*W13 + S4*W33;
    W24 = C4*W33 - S4*W13;
    W34 = QP4 - W23;
    WP14 = C4*WP13 + QP4*W24;
    WP24 = -QP4*W14 - S4*WP13;
    DV14 = W14^2;
    DV24 = W14*W24;
    DV34 = W14*W34;
    DV44 = W24^2;
    DV54 = W24*W34;
    DV64 = W34^2;
    U114 = -DV44 - DV64;
    U214 = DV24 - WP23;
    U314 = DV34 - WP24;
    U124 = DV24 + WP23;
    U224 = -DV14 - DV64;
    U324 = DV54 + WP14;
    U134 = DV34 + WP24;
    U234 = DV54 - WP14;
    U334 = -DV14 - DV44;
    VSP14 = -DV23*r4 + VP13;
    VSP24 = -U223*r4 + VP23;
    VSP34 = -U323*r4 + VSP33;
    VP14 = C4*VSP14 + S4*VSP34;
    VP24 = C4*VSP34 - S4*VSP14;
    W15 = C5*W14 - S5*W34;
    W25 = -C5*W34 - S5*W14;
    W35 = QP5 + W24;
    WP15 = C5*WP14 + QP5*W25 + S5*WP23;
    WP25 = C5*WP23 - QP5*W15 - S5*WP14;
    DV15 = W15^2;
    DV25 = W15*W25;
    DV35 = W15*W35;
    DV45 = W25^2;
    DV55 = W25*W35;
    DV65 = W35^2;
    U115 = -DV45 - DV65;
    U215 = DV25 + WP24;
    U315 = DV35 - WP25;
    U125 = DV25 - WP24;
    U225 = -DV15 - DV65;
    U325 = DV55 + WP15;
    U135 = DV35 + WP25;
    U235 = DV55 - WP15;
    U335 = -DV15 - DV45;
    VP15 = C5*VP14 + S5*VSP24;
    VP25 = C5*VSP24 - S5*VP14;
    W16 = C6*W15 + S6*W35;
    W26 = C6*W35 - S6*W15;
    W36 = QP6 - W25;
    WP16 = C6*WP15 + QP6*W26 + S6*WP24;
    WP26 = C6*WP24 - QP6*W16 - S6*WP15;
    DV16 = W16^2;
    DV26 = W16*W26;
    DV36 = W16*W36;
    DV46 = W26^2;
    DV56 = W26*W36;
    DV66 = W36^2;
    U116 = -DV46 - DV66;
    U216 = DV26 - WP25;
    U316 = DV36 - WP26;
    U126 = DV26 + WP25;
    U226 = -DV16 - DV66;
    U326 = DV56 + WP16;
    U136 = DV36 + WP26;
    U236 = DV56 - WP16;
    U336 = -DV16 - DV46;
    VP16 = C6*VP15 + S6*VP24;
    VP26 = C6*VP24 - S6*VP15;
    PSI31 = QP1*ZZR1;
    F12 = DV22*MY2 + MXR2*U112;
    F22 = DV22*MXR2 + MY2*U222;
    F32 = MXR2*U312 + MY2*U322;
    PSI12 = QP2*XZR2 + W12*XXR2 + W22*XY2;
    PSI22 = QP2*YZ2 + W12*XY2;
    PSI32 = QP2*ZZR2 + W12*XZR2 + W22*YZ2;
    No12 = -PSI22*QP2 + PSI32*W22 + WP12*XXR2 + WP22*XY2;
    No22 = PSI12*QP2 - PSI32*W12 + WP12*XY2;
    No32 = -PSI12*W22 + PSI22*W12 + WP12*XZR2 + WP22*YZ2;
    F13 = DV23*MYR3 + MX3*U113;
    F23 = DV23*MX3 + MYR3*U223;
    F33 = MX3*U313 + MYR3*U323;
    PSI13 = W13*XXR3 + W23*XY3 + W33*XZ3;
    PSI23 = W13*XY3 + W33*YZ3;
    PSI33 = W13*XZ3 + W23*YZ3 + W33*ZZR3;
    No13 = -PSI23*W33 + PSI33*W23 + WP13*XXR3 + WP23*XY3;
    No23 = PSI13*W33 - PSI33*W13 + WP13*XY3;
    No33 = -PSI13*W23 + PSI23*W13 + WP13*XZ3 + WP23*YZ3;
    F14 = MX4*U114 + MYR4*U124;
    F24 = MX4*U214 + MYR4*U224;
    F34 = MX4*U314 + MYR4*U324;
    PSI14 = W14*XXR4 + W24*XY4 + W34*XZ4;
    PSI24 = W14*XY4 + W34*YZ4;
    PSI34 = W14*XZ4 + W24*YZ4 + W34*ZZR4;
    No14 = -PSI24*W34 + PSI34*W24 + WP14*XXR4 - WP23*XZ4 + WP24*XY4;
    No24 = PSI14*W34 - PSI34*W14 + WP14*XY4 - WP23*YZ4;
    No34 = -PSI14*W24 + PSI24*W14 + WP14*XZ4 - WP23*ZZR4 + WP24*YZ4;
    F15 = MX5*U115 + MYR5*U125;
    F25 = MX5*U215 + MYR5*U225;
    F35 = MX5*U315 + MYR5*U325;
    PSI15 = W15*XXR5 + W25*XY5 + W35*XZ5;
    PSI25 = W15*XY5 + W35*YZ5;
    PSI35 = W15*XZ5 + W25*YZ5 + W35*ZZR5;
    No15 = -PSI25*W35 + PSI35*W25 + WP15*XXR5 + WP24*XZ5 + WP25*XY5;
    No25 = PSI15*W35 - PSI35*W15 + WP15*XY5 + WP24*YZ5;
    No35 = -PSI15*W25 + PSI25*W15 + WP15*XZ5 + WP24*ZZR5 + WP25*YZ5;
    F16 = MX6*U116 + MY6*U126;
    F26 = MX6*U216 + MY6*U226;
    F36 = MX6*U316 + MY6*U326;
    PSI16 = W16*XXR6 + W26*XY6 + W36*XZ6;
    PSI26 = W16*XY6 + W36*YZ6;
    PSI36 = W16*XZ6 + W26*YZ6 + W36*ZZ6;
    No16 = -PSI26*W36 + PSI36*W26 + WP16*XXR6 - WP25*XZ6 + WP26*XY6;
    No26 = PSI16*W36 - PSI36*W16 + WP16*XY6 - WP25*YZ6;
    No36 = -PSI16*W26 + PSI26*W16 + WP16*XZ6 - WP25*ZZ6 + WP26*YZ6;
    E16 = F16 + FX6;
    E26 = F26 + FY6;
    E36 = F36 + FZ6;
    N16 = CX6 - MY6*VP25 + No16;
    N26 = CY6 + MX6*VP25 + No26;
    N36 = CZ6 + MX6*VP26 - MY6*VP16 + No36;
    FDI16 = C6*E16 - E26*S6;
    FDI36 = C6*E26 + E16*S6;
    E15 = F15 + FDI16 + FX5;
    E25 = -E36 + F25 + FY5;
    E35 = F35 + FDI36 + FZ5;
    N15 = C6*N16 + CX5 + MYR5*VP24 - N26*S6 + No15;
    N25 = CY5 - MX5*VP24 - N36 + No25;
    N35 = C6*N26 + CZ5 + MX5*VP25 - MYR5*VP15 + N16*S6 + No35;
    FDI15 = C5*E15 - E25*S5;
    FDI35 = -C5*E25 - E15*S5;
    E14 = F14 + FDI15 + FX4;
    E24 = E35 + F24 + FY4;
    E34 = F34 + FDI35 + FZ4;
    N14 = C5*N15 + CX4 - MYR4*VSP24 - N25*S5 + No14;
    N24 = CY4 + MX4*VSP24 + N35 + No24;
    N34 = -C5*N25 + CZ4 + MX4*VP24 - MYR4*VP14 - N15*S5 + No34;
    FDI14 = C4*E14 - E24*S4;
    FDI34 = C4*E24 + E14*S4;
    E13 = F13 + FDI14 + FX3;
    E23 = -E34 + F23 + FY3;
    E33 = F33 + FDI34 + FZ3;
    N13 = C4*N14 + CX3 - FDI34*r4 + MYR3*VSP33 - N24*S4 + No13;
    N23 = CY3 - MX3*VSP33 - N34 + No23;
    N33 = C4*N24 + CZ3 + FDI14*r4 + MX3*VP23 - MYR3*VP13 + N14*S4 + No33;
    FDI13 = C3*E13 - E23*S3;
    FDI23 = C3*E23 + E13*S3;
    E12 = F12 + FDI13 + FX2;
    E22 = F22 + FDI23 + FY2;
    E32 = E33 + F32 + FZ2;
    N12 = C3*N13 + CX2 - FDI23*r3 - N23*S3 + No12;
    N22 = C3*N23 + CY2 - E33*d3 + FDI13*r3 + N13*S3 + No22;
    N32 = CZ2 + FDI23*d3 + MXR2*VP22 - MY2*VP12 + N33 + No32;
    FDI12 = C2*E12 - E22*S2;
    FDI32 = C2*E22 + E12*S2;
    E11 = FDI12 + FX1;
    E21 = -E32 + FY1;
    E31 = FDI32 + FZ1;
    N11 = C2*N12 + CX1 - N22*S2;
    N21 = CY1 - N32;
    N31 = C2*N22 + CZ1 + N12*S2;
   
    
    % Resultat
    c(1) = N31;
    c(2) = N32;
    c(3) = N33;
    c(4) = N34;
    c(5) = N35;
    c(6) = N36;

    
end
