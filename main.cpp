#include <iostream>
#include <armadillo>
#include <vector>

#include "modes_magnitude.h"
#include "rayleigh_ritz_beam.h"

#include "gnuplot-iostream.h"

int main(int argc, char *argv[])
{
    // Axial and bending dofs
    int nu = 4, nv = 4, nw = 4;

    // Spatial beam using Rayleigh-Ritz 
    RayleighRitzBeam needle(nu, nv, nw);

}