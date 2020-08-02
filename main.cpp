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

    // State 
    arma::dvec roa_g_g = {0.0, 0.0, 0.0};
    arma::dvec theta = {0.0, 0.0, 0.0};
    arma::dvec qf = arma::zeros<arma::dvec>(nu + nv + nw);

    arma::dvec q0 = arma::join_vert(roa_g_g, theta, qf);

    // State dot
    arma::dvec roa_dot_g_g = {0.0, 0.0, 0.0};
    arma::dvec theta_dot = {0.0, 0.0, 0.0};
    arma::dvec qf_dot = arma::zeros<arma::dvec>(nu + nv + nw);

    arma::dvec q0_dot = arma::join_vert(roa_dot_g_g, theta_dot, qf_dot);

    needle.update(0, q0, q0_dot);

    std::cout << needle.get_stiffness_matrix() << std::endl;

}