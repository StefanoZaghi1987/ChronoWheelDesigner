%{

/* Includes the header in the wrapper code */
#include "chrono/physics/ChShaftsTorsionSpring.h"

%}
 
// Tell SWIG about parent class in Python
%import "ChShaftsCouple.i"


/* Parse the header file to generate wrappers */
%include "../chrono/physics/ChShaftsTorsionSpring.h"  







