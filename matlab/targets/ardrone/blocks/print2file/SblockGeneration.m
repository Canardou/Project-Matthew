%Ce script sert � cr�er un bloc S-function �partir d'une fonction C !

def = legacy_code('initialize')
% Nom du fichier et du header
def.SourceFiles = {'print2file.cpp'};
def.HeaderFiles = {'print2file.h'};
def.IncPaths = {'.',};

% Nom de la fonction
def.SFunctionName = 'ARDrone_Print2File';
% Les sorties sont y1, y2... etc, 
% les entr�es sont u1, u2...etc
% les param�tres sont p1, p2 ...
def.Options.language = 'C++'
def.StartFcnSpec = 'void print2file_Init(int8 p1[])';
def.OutputFcnSpec = 'void print2file(int32 u1)';
%def.StopFcnSpec = 'print2fileInit_Close()';

%YOLO
legacy_code('sfcn_cmex_generate', def);
legacy_code('compile', def);

%Genere le block sous simulink
legacy_code('slblock_generate', def);
legacy_code('sfcn_tlc_generate', def);
legacy_code('rtwmakecfg_generate', def);
