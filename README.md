# ERGToolbox v1.0
MATLAB based toolbox for the synthesis of Explicit Reference Governors (ERG). For further information on the project, please visit [the site of the project](https://saasofcc.wordpress.com/)
## Overview
The ERGT provides an user friendly way to design and simulate Explicit Reference Governors. In its current version, the ERGT supports linear systems subject to linear state and input constraints and linear systems subject to union of linear state and input constraints.
## How to install
### Prequisites
ERGT requires the installation of [MPT3](http://people.ee.ethz.ch/~mpt/3/). Additionally, the installation of [SDPT3](http://www.math.nus.edu.sg/~mattohkc/sdpt3.html) is advised, although not necessary.
### Installation
To install the ERG Toolbox, just run the script named `ERGTinstall.m`. Choose in the window the directory in which ERGT will be installed, the script will copy all necessary files and will add the directory to MATLAB's path. Alternatively, the user can copy all files to a location of their choosing and subsequently add the location to MATLAB's path.
## Citing ERGT
If you use the ERG Toolbox, please use the following BibTeX entry:
```
@INPROCEEDINGS{Coto1809:Explicit,
AUTHOR="Andr{\'e}s {Cotorruelo Jim{\'e}nez} and Daniel Limon and Marco Nicotra and
Emanuele Garone",
TITLE="Explicit Reference Governor Toolbox {(ERGT)}",
BOOKTITLE="2018 IEEE 4th International Forum on Research and Technology for Society
and Industry (RTSI) (RTSI 2018)",
ADDRESS="Palermo, Italy",
DAYS=9,
MONTH=sep,
YEAR=2018,
ABSTRACT="This paper presents the first release of the Explicit Reference Governor
Toolbox (ERGT). The ERGT aims at providing an user-friendly interface for
the design of an ERG scheme. The ERGT includes several built-in functions
for the design phase, as well as some Simulink blocks to help with the
simulation. In this current stage, this toolbox is aimed towards the design
of ERG control systems for linear systems subject to the union and
intersection of linear constraints. The paper presents a simple example on
an aerial robotic application."
}
```
