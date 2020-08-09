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
    arma::dvec calculate( arma::dvec state_vector, double t);

    // Get reaction forces 
    arma::dvec get_reaction_forces(void) { return m_fc_f; }

    // Get reaction forces 
    arma::dvec get_reaction_moment(void) { return m_mc_f; }

    // Get model size 
    uint get_model_size(void) {return 2 * m_dofs; }

private:

    // Number of dofs
    uint m_dofs;

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


};

