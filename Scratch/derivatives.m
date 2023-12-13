clc; clear all; close all;

syms t 'positive'

P = [-0.05*(t+3)^2+5;
          -5+.5*t;
        -2*sin(t/8)+0.5*cos(t/2) + 5;];

V = diff(P, t)

A = diff(V, t)
