## Ball on beam balancer ##
This repository contains code and tools for [my DIY ball-on-beam balancer.](https://dvernooy.github.io/projects/bobb)

#### v0.1 ####
- /excel folder contains a PID model for the system as well as a spreadsheet to explore kalman filters
- /matlab_octave contains some .m files for determining the parameters for the controller
- /source/ contains common files for the build based on ATMega328
- /source/pid contains pid-specific main.c
- /source/state-space contains main.c for both the kalman and pole-placement approaches to the state-space observer
