#pragma once 

#include <iostream>
#include <armadillo>
#include "needle_properties.hpp"
#include "dynamics_math.h"
#include "modes_magnitude.h"
#include "euler_rotations.h"
#include "state.h"

class RayleighRitzBeam : public NeedleProperties
{
public:
    RayleighRitzBeam(uint axial_dofs, uint bending_y_dofs, uint bending_z_dofs);

//     // Calculate system model function 
//     arma::dvec calculate(arma::dvec state_vector, double t);

    // Mass matrix getter
    arma::dmat get_mass_matrix(void){ return m_mass; }
    
    // Stiffness matrix getter
    arma::dmat get_stiffness_matrix(void){ return m_stiffness; }

//     // External force getter 
//     arma::dvec get_external_force(void){ return m_qforce; }

//     // Get system size 
//     uint get_model_size(void) { return m_dofs; }

//     // Get system deflection 
//     arma::dvec get_deflection(double ksi, arma::dvec qf);

//     // Get beam length
//     double get_beam_length(void) { return m_beam_length; }

    // Update flexible body matrices and coriolis vector
    void update(double t, arma::dvec q, arma::dvec q_dot);

private:
    // Number of axial dofs 
    uint m_axial_dofs;

    // Number of bending dofs y direction
    uint m_bending_y_dofs;

    // Number of bending dofs z direction
    uint m_bending_z_dofs;

    // Number of dofs
    uint m_dofs;

private:
    // Beam lenght (m)
    double m_beam_length;

    // Beam radius (m)
    double m_beam_radius;

    // Beam cross-sectional area (m^2)
    double m_beam_area;

    // Beam mass (kg)    
    double m_beam_mass;
    
    // Beam area moment of inertia iyy (m^4)
    double m_iyy;

    // Beam area moment of inertia izz (m^4)
    double m_izz;

    // Beam young modulus (N / m^2)
    double m_beam_young_modulus;

    // Beam density (kg / m^2)
    double m_beam_density;

    // Distance of body's centre of mass from reference frame (m)
    arma::dvec m_raco_f_f;

    // Inertial tensor of body wrt to the centre of mass (reference frame) (kg * m^2)
    arma::dmat m_i_co_f;

    // Inertial tensor of body wrt to point A (reference frame) (kg * m^2)
    arma::dmat m_i_a_f;

    // Gravity acceleration (m/s^2)
    const double m_grav = 9.80665;

private:
    // Flexible body mass matrix
    arma::dmat m_mass;

    // Flexible body stiffness matrix
    arma::dmat m_stiffness;

    // Coriolis-Centrifugal vector
    arma::dvec m_fvf;

    // Flexible body external force 
    arma::dvec m_qforce;

private:
    // State 
    arma::dvec m_roa_g_g, m_theta, m_qf, m_omega;

    // State dot 
    arma::dvec m_roa_dot_g_g, m_theta_dot, m_qf_dot;

    // G and Gdot matrices
    arma::dmat m_g_mat, m_g_dot_mat;

    // Rotation matrix 
    arma::dmat m_rot_f_F;

private:
    // Mass matrix of flexible body calculation 
    void mass_matrix_calculation(void); 

    // Stiffness matrix of flexible body calculation 
    void stiffness_matrix_calculation(void); 

    // Coriolis-centrifugal forces calculation
    void coriolis_vector_calculation(void);

    // // External force calculation
    // void external_force_calculation(double t, arma::dvec q, arma::dvec q_dot);


private:

    // Shape integrals dash
    arma::dmat m_phi11_dash, m_phi12_dash, m_phi13_dash;
    arma::dmat m_phi21_dash, m_phi22_dash, m_phi23_dash;
    arma::dmat m_phi31_dash, m_phi32_dash, m_phi33_dash;

    // Shape integrals hat
    arma::drowvec m_phi11_hat, m_phi12_hat, m_phi13_hat;
    arma::drowvec m_phi21_hat, m_phi22_hat, m_phi23_hat;
    arma::drowvec m_phi31_hat, m_phi32_hat, m_phi33_hat;

    // N integrals
    arma::dmat m_n_int[12];

    // Locator vectors
    arma::ivec m_lu, m_lv, m_lw;

    // Locator matrices
    arma::dmat m_lu_mat, m_lv_mat, m_lw_mat; 

    // // Shape function 
    // arma::dmat shape_function(double x);

private:
    // Natural frequency y dircetion
    arma::dvec m_sy = {1.8751, 4.694, 7.8547, 10.9955};

    // Natural frequency z dircetion
    arma::dvec m_sz = {1.8751, 4.694, 7.8547, 10.9955};

    // Wave coefficient 
    double m_c;

    // Axial frequencies (rad / sec)
    arma::dvec m_u_freq;

    // Bending y frequencies (rad / sec)
    arma::dvec m_v_freq;

    // Bending z frequencies (rad / sec)
    arma::dvec m_w_freq;
    
    // Axial mode coefficient
    arma::dvec m_alpha_freq;

    // Bending y mode coefficient
    arma::dvec m_beta_freq;

    // Bending z mode coefficient
    arma::dvec m_gamma_freq;

    // Axial mode magnitude
    arma::dvec m_u_hat;

    // Bending y mode magnitude
    arma::dvec m_v_hat;

    // Bending z magnitude
    arma::dvec m_w_hat;

    // Bending y mode magnitude coefficient
    arma::dvec m_jvn;

    // Bending z mode magnitude coefficient
    arma::dvec m_jwn;

// private:
//     // External force body frame (position l)
//     arma::dvec external_force(double t, arma::dvec q, arma::dvec q_dot);

//     // External traction force
//     arma::dvec external_traction_force(double t, arma::dvec q, arma::dvec q_dot);

//     // Distributed load body frame
//     arma::dvec distributed_load(double t, double x, arma::dvec q, arma::dvec q_dot);

private:
    // R integrals 
    double r_integrals(double a, double b, double l, int id);

    // Shape integrals calculation
    void shape_integrals(void);

    // N integrals calculation
    void n_integrals(void);
};
