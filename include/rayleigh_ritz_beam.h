#pragma once 

#include <iostream>
#include <armadillo>
#include "needle_properties.hpp"
#include "dynamics_math.h"
#include "modes_magnitude.h"
#include "euler_rotations.h"

class RayleighRitzBeam : public NeedleProperties
{
public:
    RayleighRitzBeam(uint axial_dofs, uint bending_y_dofs, uint bending_z_dofs);

    // Mass matrix getter
    arma::dmat get_mass_matrix(void){ return m_mass; }
    
    // Damping matrix getter
    arma::dmat get_damping_matrix(void){ return m_damping; }

    // Stiffness matrix getter
    arma::dmat get_stiffness_matrix(void){ return m_stiffness; }

    // Coriolis vector getter
    arma::dvec get_coriolis_vector(void){ return m_fvf; }

    // External force getter 
    arma::dvec get_external_force(void){ return m_qforce; }

    // Get system deflection 
    arma::dvec get_deflection(double ksi, arma::dvec qf);

    // Get beam length
    double get_beam_length(void) { return m_beam_length; }

    // Update flexible body matrices and coriolis vector
    void update(double t, arma::dvec q, arma::dvec q_dot);

    // Get elastic dofs 
    uint get_elastic_dofs(void) { return m_elastic_dofs; }

    // Get axial frequencies 
    arma::dvec get_axial_frequencies(void) { return m_u_freq; }

    // Get bending y frequencies 
    arma::dvec get_bending_y_frequencies(void) { return m_v_freq; }

    // Get bending z frequencies 
    arma::dvec get_bending_z_frequencies(void) { return m_w_freq; }

private:
    // Number of axial dofs 
    uint m_axial_dofs;

    // Number of bending dofs y direction
    uint m_bending_y_dofs;

    // Number of bending dofs z direction
    uint m_bending_z_dofs;

    // Number of elastic dofs
    uint m_elastic_dofs;

    // Number of rigid dofs 
    const uint m_rigid_dofs = 6;

    // Total number of dofs
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

    // Weight wrt to inertial frame
    arma::dvec m_weight_F;

    // Damping coeefficients 
    const double m_mu = 10.0;
    const double m_kappa = 0.0;

private:
    // Flexible body mass matrix
    arma::dmat m_mass;

    // Flexible body damping matrix
    arma::dmat m_damping;

    // Flexible body stiffness matrix
    arma::dmat m_stiffness;

    // Coriolis-Centrifugal vector
    arma::dvec m_fvf;

    // Flexible body external force 
    arma::dvec m_qforce;

public:
    arma::dmat get_mf31_matrix(void){ return m_mf31; }
    arma::dmat get_mf32_matrix(void){ return m_mf32; }
    arma::dmat get_mf33_matrix(void){ return m_mf33; }
    arma::dmat get_cf33_matrix(void){ return m_cf33; }
    arma::dmat get_kf33_matrix(void){ return m_kf33; }
    arma::dmat get_fvf3_vector(void){ return m_fvf3; }
    arma::dmat get_qf3_vector(void){ return m_qf3; }

private:
    // Elastic components
    // Mass matrix components 
    arma::dmat m_mf31, m_mf32, m_mf33;

    // Damping matrix component
    arma::dmat  m_cf33;

    // Stiffness matrix component
    arma::dmat  m_kf33;

    // Coriolis vector component 
    arma::dvec m_fvf3; 

    // External forces component 
    arma::dvec m_qf3; 

private:
    // State 
    arma::dvec m_roa_g_g, m_theta, m_qf, m_omega;

    // State dot 
    arma::dvec m_roa_dot_g_g, m_theta_dot, m_qf_dot;

    // G and Gdot matrices
    arma::dmat m_g_mat, m_g_dot_mat;

    // Rotation matrix 
    arma::dmat m_rot_f_F;

    // Current time 
    double m_time;
    
private:
    // State update 
    void state_update(arma::dvec q, arma::dvec q_dot);

    // Mass matrix of flexible body calculation 
    void mass_matrix_calculation(void); 

    // Stiffness matrix of flexible body calculation 
    void damping_matrix_calculation(void); 

    // Stiffness matrix of flexible body calculation 
    void stiffness_matrix_calculation(void); 

    // Coriolis-centrifugal forces calculation
    void coriolis_vector_calculation(void);

    // External force calculation
    void external_force_calculation(void);

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

private:
    // External force body frame (position l) FB(t, q, q_dot)
    arma::dvec external_force(void);

    // Distributed load body frame p(t, x, q , q_dot)
    arma::dvec distributed_load(double x);

private:

    // Shape function 
    arma::dmat shape_function(double x);

    // R integrals 
    double r_integrals(double a, double b, double l, int id);

    // Shape integrals calculation
    void shape_integrals(void);

    // N integrals calculation
    void n_integrals(void);

    // D(x) function
    arma::dmat d_x(double x);
};
