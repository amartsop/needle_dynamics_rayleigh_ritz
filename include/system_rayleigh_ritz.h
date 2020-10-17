#pragma once 

#include <iostream>

#include "handle.h"
#include "euler_rotations.h"
#include "rayleigh_ritz_beam.h"
#include "input_trajectory.h"


class SystemRayleighRitz
{
public:
    SystemRayleighRitz(Handle *handle, RayleighRitzBeam *needle,
        InputTrajectory *input_traj);

    // Calculate system model function 
    arma::dvec f(double t, const arma::dvec& state_vector);

    // Calculate system's Jacobian
    arma::dmat dfdx(double t, const arma::dvec& x);

    // Get reaction forces 
    arma::dvec get_reaction_forces(void) const { return m_fc_f; }

    // Get reaction forces 
    arma::dvec get_reaction_moment(void) const { return m_mc_f; }

private:

    // Number of elastic dofs
    uint m_elastic_dofs;

    // Raction force 
    arma::dvec m_fc_f;

    // Raction Moment
    arma::dvec m_mc_f;

private:
    // Rigid body (handle)
    Handle* m_handle_ptr;

    // Flexible body (needle)
    RayleighRitzBeam* m_needle_ptr; 

    // Input coordinates 
    InputTrajectory* m_input_traj_ptr;

private:
    // Numerical Jacobian tolerance
    double m_tol = 1.0e-8;

};

