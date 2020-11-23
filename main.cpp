#include <cstdlib>
#include <iostream>
#include <armadillo>
#include <ostream>
#include <vector>
#include <chrono>
#include <thread>

#include <modes_magnitude.h>
#include <input_trajectory.h>
#include <handle.h>
#include <rayleigh_ritz_beam.h>
#include <system_rayleigh_ritz.h>
#include <numerical_integration.hpp>

#include <animation.h>

int main(int argc, char *argv[])
{
    // Axial and bending dofs
    int nu = 1, nv = 2, nw = 2;

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
    arma::dvec qf0 = arma::zeros<arma::dvec>(needle.get_elastic_dofs());
    arma::dvec qf0_dot = arma::zeros<arma::dvec>(needle.get_elastic_dofs());

    // State vector initialization
    std::vector<arma::dvec> state_vector;
    arma::dvec state0 = arma::join_vert(qf0, qf0_dot);
    state_vector.push_back(state0);

    // Timing
    double t_final = 10.0; // Final time (s)
    double fs = 1e3;  // Simulation frequency (Hz)
    double h = 1.0 / fs; // Integration time step (s)
    double t = 0; // Initial time (s) 
    
    // Time vector initialization 
    std::vector<double> time_vector;
    time_vector.push_back(t);

    // Problem solver 
    NumericalIntegration<SystemRayleighRitz> ni(&system_rayleigh_ritz, h,
        state0.n_rows);

    // Iteration counter 
    unsigned int counter = 0;
	
    while (t <= t_final)
    {
        // System solution 
        arma::dvec x = ni.implicit_euler(t, state_vector.at(counter));
        state_vector.push_back(x);

        if (!x.is_finite()) { std::cout << "Error" << std::endl; break; };

        // Update time vector 
        time_vector.push_back(t);

        // Update time and counter
        t += h; counter += 1;

        std::cout << t << std::endl;
    }

    // Animation
    Animation animation;
    double animation_frequency = 30; // (Hz) Frames per sec
    double animation_period_sec = 1 / animation_frequency; // (s)
    uint animation_period =  1000 * animation_period_sec; // (ms)
    uint steps = fs / animation_frequency;

    // Clock
    auto start_time = std::chrono::steady_clock::now();
    double real_time = 0;

    // Needle vertices
    auto needle_vert = animation.getNeedleVerticesPos();
    std::vector<glm::vec3> needle_vert_new; needle_vert_new.resize(needle_vert.size());

    // State counter 
    size_t state_counter = 0;

    /*********************** Render Loop ***********************/
    for(;;)
    {
        // Break animation
        if(animation.isDone()) { break; }

        // Time tracking
        auto last_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = last_time - start_time;

        // Current state and time 
        arma::dvec x_current = state_vector.at(state_counter);
        double t_current = time_vector.at(state_counter);

        /******************** Update handle pose ********************/
        // Known coordinates 
        input_traj.update(t_current);

        // Rigid body position and orientation
        arma::dvec roa_F_F = {0.0f, 0.0f, 0.0f};
        arma::dvec euler_angles = {0.0f, 0.0f, 0.0f};

        /******************** Update needle pose and shape ********************/
        // Elastic coordinates
        arma::dvec qf = x_current.rows(0, needle.get_elastic_dofs() - 1);

        // Reset needle vertices
        for (unsigned int i = 0; i < needle_vert.size(); i++)
        {
            glm::vec3 needle_vert_i = needle_vert.at(i);

            arma::dvec r_ap0_f_f = {needle_vert_i.x, needle_vert_i.y,
                needle_vert_i.z};

            arma::dvec defl = needle.get_deflection(r_ap0_f_f, qf);

            arma::dvec total_defl = r_ap0_f_f + defl;

            needle_vert_new.at(i) = glm::vec3(total_defl(0), total_defl(1),
                total_defl(2));
        }
        animation.setNeedleVerticesPos(needle_vert_new);

        // Render animation 
        animation.update(elapsed_seconds.count());

        // Delay
        std::this_thread::sleep_for(std::chrono::milliseconds(animation_period));

        // Update time 
        start_time = last_time;

        // Update state counter 
        if (state_counter < state_vector.size() - 1) { state_counter += steps ; }
        else { break;}
    }
}
    




