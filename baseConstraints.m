function [E,F,G]=baseConstraints(lanewidth,lanes)
    E1=[0 0 0];
    F1=[0 0 1 0];

    G1=lanewidth*lanes/2;
    % 
    E2=[0 0 0];
    F2=[0 0 -1 0];
    G2=lanewidth*lanes/2;
    %
    E3=[0 0 0];
    F3=[0 0 1 0];
    G3=lanewidth*lanes/2;
    E=[E1;E2;E3];
    F=[F1;F2;F3];
    G=[G1;G2;G3];
end