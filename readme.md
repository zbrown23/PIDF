# PIDF

PIDF is a simple implementation of proportional-integral-derivative-feedforward controller in C++17. It includes anti-integral windup and bandwidth limiting on the D term.

## Features

PIDF makes use of C++'s templating system to allow a user to use any type that resembles a real number within the controller itself. This means that a unit library's type can be used, instead of having
to represent the system in raw `float` or `double`. It also means that you can specify fixed-point or single precision floating point numbers in the case that the application does not have a DPFPU or 
any FPU at all.


