#include <iostream>
#include <armadillo>
#include <vector>
#include <chrono>
#include <thread>

#include "gnuplot-iostream.h"
#include "modes_magnitude.h"
#include "input_trajectory.h"
#include "handle.h"
#include "rayleigh_ritz_beam.h"
#include "system_rayleigh_ritz.h"

#include "needle_animation.hpp"
#include "numerical_integration.hpp"
#include "post_processing_rr.hpp"


int main(int argc, char *argv[])
{
    // Axial and bending dofs
    int nu = 1, nv = 4, nw = 4;

    // Input trajectory
    InputTrajectory input_traj;

    // Rigid body 
    Handle handle;

    // Spatial beam using Rayleigh-Ritz 
    RayleighRitzBeam needle(nu, nv, nw);

    // Handle and beam system
    SystemRayleighRitz system_rayleigh_ritz(&handle, &needle, &input_traj);


    // /********************* Simulation ************************/ 
    // Initial beam deflection 
    arma::dvec qf0 = arma::zeros<arma::dvec>(nu + nv + nw);
    arma::dvec qf0_dot = arma::zeros<arma::dvec>(nu + nv + nw);

    // State vector initialization
    std::vector<arma::dvec> state_vector;
    arma::dvec state0 = arma::join_vert(qf0, qf0_dot);
    state_vector.push_back(state0);

    // Reaction forces and moments
    std::vector<double> fx, fy, fz;
    std::vector<double> mx, my, mz;

    // Timing
    double t_final = 5.0; // Final time (s)
    double fs = 1e3;  // Simulation frequency (Hz)
    double h = 1.0 / fs; // Integration time step (s)
    double t = 0; // Initial time (s) 
    
    // Time vector initialization 
    std::vector<double> time_vector;
    time_vector.push_back(t);

    // Post processing 
    PostProcessingRR<RayleighRitzBeam> post_processing_rr(&needle);

    // Problem solver 
    NumericalIntegration<SystemRayleighRitz> ni(&system_rayleigh_ritz, h,
        state0.n_rows);

    // Iteration counter 
    uint counter = 0;

    while (t <= t_final)
    {
        // System solution 
        arma::dvec x = ni.implicit_midpoint(t, state_vector.at(counter));
        state_vector.push_back(x);

        if (!x.is_finite()) {
            std::cout << "Error" << std::endl; 
            break; 
        };

        // Reaction forces 
        arma::dvec reaction_forces = - system_rayleigh_ritz.get_reaction_forces();
        fx.push_back(reaction_forces(0)); fy.push_back(reaction_forces(1));
        fz.push_back(reaction_forces(2));

        // Reaction moment
        arma::dvec reaction_moment = - system_rayleigh_ritz.get_reaction_moment();
        mx.push_back(reaction_moment(0)); my.push_back(reaction_moment(1));
        mz.push_back(reaction_moment(2));

        // Update time vector 
        time_vector.push_back(t);

        // Update time and counter
        t += h; counter += 1;

        std::cout << t << std::endl;
    }

    // Plot reaction forces 
    Gnuplot gp;
    arma::dvec t_vec(time_vector);
    arma::dvec fx_vec(fx);
    arma::dvec fy_vec(fy);
    arma::dvec fz_vec(fz);

    arma::dvec mx_vec(mx);
    arma::dvec my_vec(my);
    arma::dvec mz_vec(mz);

    arma::dmat t_fx = arma::join_horiz(t_vec.rows(0, t_vec.n_rows - 2), fx_vec);
    arma::dmat t_fy = arma::join_horiz(t_vec.rows(0, t_vec.n_rows - 2), fy_vec);
    arma::dmat t_fz = arma::join_horiz(t_vec.rows(0, t_vec.n_rows - 2), fz_vec);

    arma::dmat t_mx = arma::join_horiz(t_vec.rows(0, t_vec.n_rows - 2), mx_vec);
    arma::dmat t_my = arma::join_horiz(t_vec.rows(0, t_vec.n_rows - 2), my_vec);
    arma::dmat t_mz = arma::join_horiz(t_vec.rows(0, t_vec.n_rows - 2), mz_vec);

    gp << "plot '-' with lines \n";
    gp.send1d(t_my);


    // Animation
    NeedleAnimation needle_animation(&needle, &post_processing_rr);
    double animation_frequency = 30; // (Hz) Frames per sec
    double animation_period_sec = 1 / animation_frequency; // (s)
    uint animation_period =  1000 * animation_period_sec; // (ms)
    uint steps = fs / animation_frequency;

    // Clock
    auto start = std::chrono::steady_clock::now();
    double real_time = 0;

    for(size_t i = 0; i <= time_vector.size(); i = i + steps)
    {
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;

        // Current state and time 
        arma::dvec x_current = state_vector.at(i);
        double t_current = time_vector.at(i);

        // Known coordinates 
        input_traj.update(t_current);

        // Rigid body position and orientation
        arma::dvec roa_F_F = input_traj.get_linear_displacement();
        arma::dvec euler_angles = input_traj.get_rotational_displacement();

        // Elastic coordinates
        arma::dvec qf = x_current.rows(0, needle.get_elastic_dofs() - 1);

        // Animation
        needle_animation.animate(roa_F_F, euler_angles, qf);

        // Delay
        std::this_thread::sleep_for(std::chrono::milliseconds(animation_period));

        // Calculate Real time
        double real_time = real_time + elapsed_seconds.count();

        // Update time
        start = end;
    }

}