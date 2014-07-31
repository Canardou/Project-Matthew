Project-Matthew - AHRS
===============
###AHRS implementation from Project-Matthew code

__Project Matthew__ : [https://sites.google.com/site/projetsecinsa/projets-2013-2014/project-matthew](https://sites.google.com/site/projetsecinsa/projets-2013-2014/project-matthew)

###Starting

For testing purspose we used the makefile in <code>bibrone/tests</code>.
In order to use it, you must modify the path to the g++ compiler :

Ex : <code>CPP=/home/user/CodeSourcery/bin/arm-none-linux-gnueabi-g++

Then use the <code>make</code> command to generate the .elf file 
(or <code>make -B</code> if you modify any other files which is not in the tests repository.

Then use <code>ftp 192.168.1.1</code> to put the file on the drone and <code>telnet 192.168.1.1</code> to connect to it.
The file you upload are in the data/video repository.

The first time you connect to a drone, don't forget to kill the defautl program : 
use <code>ps</code> and <code>kill n°task</code> wich have program.elf as name.

If you modify the ahrs.h file, it must be the same in bibrone/src and
bibrone/include/bibrone.

###How to use the AHRS

Simply use like following example :

```
ahrs test;
test.Initialize();
test.Start(0.02);
...
test.GetEuler();
```
or

```
ahrs test;
test.Initialize();
test.SetQuaternion(false);
test.Set(0.01,2,15);//Nécessaire (coef PI diff)
test.Start();
```
