#include "rayleigh_ritz_beam.h"


RayleighRitzBeam::RayleighRitzBeam(uint axial_dofs, uint bending_y_dofs, uint bending_z_dofs)
{
    // Number of axial dofs 
    m_axial_dofs = axial_dofs;

    // Number of bending dofs y direction
    m_bending_y_dofs = bending_y_dofs;

    // Number of bending dofs z direction
    m_bending_z_dofs = bending_z_dofs;

    // Number of dofs
    m_dofs = m_axial_dofs + m_bending_y_dofs + m_bending_z_dofs;

    // Beam length 
    m_beam_length = m_needle_length;

    // Beam length 
    m_beam_radius = m_needle_radius;

    // Cross-sectional area
    m_beam_area = M_PI * pow(m_beam_radius, 2.0);

    // Beam density
    m_beam_density = m_needle_density;

    // Beam mass (kg)    
    m_beam_mass = m_beam_density * m_beam_area * m_beam_length;

    // Beam area moment of inertia iyy
    m_iyy = (M_PI / 4) * pow(m_beam_radius, 4.0);

    // Beam area moment of inertia izz
    m_izz = (M_PI / 4) * pow(m_beam_radius, 4.0);

    // Beam young modulus
    m_beam_young_modulus = m_needle_young_modulus;

    // Distance of body's centre of mass from reference frame (m)
    m_raco_f_f = {m_beam_length / 2.0, 0.0, 0.0};

    // Inertial tensor of body wrt to the centre of mass (reference frame) 
    m_i_co_f = {{m_beam_mass * pow(m_beam_radius, 2.0) / 2.0, 0.0, 0.0},
        {0.0, m_beam_mass * (pow(m_beam_length, 2.0) +
        3.0 * pow(m_beam_radius, 2.0)) / 12.0, 0.0},
        {0.0, 0.0, m_beam_mass * (pow(m_beam_length, 2.0) +
        3.0 * pow(m_beam_radius, 2.0)) / 12.0}};

    // Wave coefficient
    m_c = sqrt(m_beam_young_modulus / m_beam_density);

    // Locator vectors
    m_lu = arma::regspace<arma::ivec>(1, 1, m_axial_dofs);
    m_lv = arma::regspace<arma::ivec>(m_axial_dofs + 1, 1,
        m_axial_dofs + m_bending_y_dofs);
    m_lw = arma::regspace<arma::ivec>(m_axial_dofs + m_bending_y_dofs + 1, 1,
        m_axial_dofs + m_bending_y_dofs + m_bending_z_dofs);

    // Locator matrices
    m_lu_mat = dm::locator_matrix(m_lu, m_dofs);
    m_lv_mat = dm::locator_matrix(m_lv, m_dofs);
    m_lw_mat = dm::locator_matrix(m_lw, m_dofs);

    // Initialization of frequencies vectors, mode coeffiients and mode magnitudes
    m_u_freq = m_alpha_freq = m_u_hat = arma::zeros(m_axial_dofs);
    m_v_freq = m_beta_freq = m_v_hat = m_jvn = arma::zeros(m_bending_y_dofs);
    m_w_freq = m_gamma_freq = m_w_hat = m_jwn = arma::zeros(m_bending_z_dofs);

    // Frequencies and coefficients assignment
    for (uint i = 0; i < m_axial_dofs; i++)
    {
        // Mode number 
        uint n = i + 1;

        // Mode frequency
        m_u_freq(i) = (2.0 * (double) n - 1.0) * (M_PI * m_c) / (2.0 * m_beam_length);

        // Mode frequency coefficient
        m_alpha_freq(i) = m_u_freq(i) / m_c;

        // Mode magnitude
        m_u_hat(i) = sqrt(2.0 / m_beam_mass);
    }
    
    for (uint i = 0; i < m_bending_y_dofs; i++)
    {
        // Mode number 
        uint n = i + 1; double sy_n = arma::as_scalar(m_sy(i));

        // Mode frequency 
        m_v_freq(i) = sqrt((m_beam_young_modulus * m_izz) 
            / (m_beam_density * m_beam_area)) * pow((sy_n / m_beam_length), 2.0);

        // Mode frequency coefficient
        m_beta_freq(i) = sy_n / m_beam_length;

        // Mode magnitude
        double rvn = ModesMagnitude::r(m_beam_length, m_beta_freq(i));
        m_v_hat(i) = 1.0 / sqrt(m_beam_density * m_beam_area * rvn);

        // Mode magnitude coefficient
        m_jvn(i) = ModesMagnitude::jn(m_beam_length, m_beta_freq(i));
    }

    for (uint i = 0; i < m_bending_z_dofs; i++)
    {

        // Mode number 
        uint n = i + 1; double sz_n = arma::as_scalar(m_sz(i));

        // Mode frequency 
        m_w_freq(i) = sqrt((m_beam_young_modulus * m_iyy) 
            / (m_beam_density * m_beam_area)) * pow((sz_n / m_beam_length), 2.0);

        // Mode frequency coefficient
        m_gamma_freq(i) = sz_n / m_beam_length;

        // Mode magnitude
        double rwn = ModesMagnitude::r(m_beam_length, m_gamma_freq(i));
        m_w_hat(i) = 1.0 / sqrt(m_beam_density * m_beam_area * rwn);

        // Mode magnitude coefficient
        m_jwn(i) = ModesMagnitude::jn(m_beam_length, m_gamma_freq(i));
    }

    // Calculate shape integrals
    shape_integrals();

    // Calculate stiffness matrix
    stiffness_matrix_calculation();
}

void RayleighRitzBeam::update(double t, arma::dvec q, arma::dvec q_dot)
{

    // State 
    m_theta = state::theta(q, q_dot);
    m_theta_dot = state::theta_dot(q, q_dot);
    m_qf = state::qf(q, q_dot);
    m_qf_dot = state::qf_dot(q, q_dot);

    // G and Gdot matrices
    m_g_mat = EulerRotations::G(m_theta);
    m_g_dot_mat = EulerRotations::G_dot(m_theta, m_theta_dot);

    // Rotation matrix 
    m_rot_f_F = EulerRotations::rotation(m_theta);

    // Angular velocity (rad / sec)
    m_omega =  m_g_mat * m_theta_dot;

    // Update N integrals
    n_integrals();

    // Update mass matrix 
    mass_matrix_calculation();

    // Update coriolis-centrifugal vector 
    coriolis_vector_calculation();
}

void RayleighRitzBeam::mass_matrix_calculation(void)
{
    // First row   
    arma::dmat mf11 = m_beam_mass * arma::eye(3, 3);

    arma::dmat mf12 = - m_rot_f_F * (m_beam_mass * dm::s(m_raco_f_f) + 
        dm::s(m_n_int[0] * m_qf)) * m_g_mat;

    arma::dmat mf13 =  m_rot_f_F * m_n_int[0];

    arma::dmat mf1 = arma::join_horiz(mf11, mf12, mf13);

    // Second row   
    arma::dmat mf21 =  mf12.t();

    arma::dmat mf22 =  m_g_mat.t() * m_i_a_f * m_g_mat;

    arma::dmat mf23 = - m_g_mat.t() * (m_n_int[4] + m_n_int[5]);

    arma::dmat mf2 = arma::join_horiz(mf21, mf22, mf23);

    // Third row   
    arma::dmat mf31 = mf13.t();

    arma::dmat mf32 = mf23.t();

    arma::dmat mf33 = m_n_int[6];

    arma::dmat mf3 = arma::join_horiz(mf31, mf32, mf33);

    // Mass matrix 
    m_mass = arma::join_vert(mf1, mf2, mf3);
}

// Stiffness matrix calculation
void RayleighRitzBeam::stiffness_matrix_calculation(void)
{
    // Axial stiffness matrix 
    arma::dmat ku = arma::zeros<arma::dmat>(m_axial_dofs, m_axial_dofs);
    for (uint i = 0; i < m_axial_dofs; i++)
    {
        ku(i, i) = pow(m_u_freq(i), 2.0);
    }

    // Bending y stiffness matrix 
    arma::dmat kv = arma::eye<arma::dmat>(m_bending_y_dofs, m_bending_y_dofs);
    for (uint i = 0; i < m_bending_y_dofs; i++)
    {
        double rv_n = ModesMagnitude::r(m_beam_length, m_beta_freq(i));
        double rv_tilde_n = ModesMagnitude::r_tilde(m_beam_length, m_beta_freq(i));
        kv(i, i) = (rv_n / rv_tilde_n) * pow(m_v_freq(i), 2.0);
    }

    // Bending z stiffness matrix 
    arma::dmat kw = arma::eye<arma::dmat>(m_bending_z_dofs, m_bending_z_dofs);
    for (uint i = 0; i < m_bending_z_dofs; i++)
    {
        uint n = i + 1; double sz_n = arma::as_scalar(m_sz(i));

        double rw_n = ModesMagnitude::r(m_beam_length, m_gamma_freq(i));
        double rw_tilde_n = ModesMagnitude::r_tilde(m_beam_length, m_gamma_freq(i));
        kw(i, i) = (rw_n / rw_tilde_n) * pow(m_w_freq(i), 2.0);
    }

    m_stiffness = m_lu_mat.t() * ku * m_lu_mat + m_lv_mat.t() * kv * m_lv_mat + 
        m_lw_mat.t() * kw * m_lw_mat;
}

// Coriolis-centrifugal vector calculation
void RayleighRitzBeam::coriolis_vector_calculation(void)
{
    // First vector
    arma::dvec fvf1 = m_rot_f_F * ((m_beam_mass * dm::s(m_raco_f_f) +
        dm::s(m_n_int[0] * m_qf)) * m_g_dot_mat * m_theta_dot -
        pow(dm::s(m_omega), 2.0) * (m_raco_f_f * m_beam_mass +
        m_n_int[0] * m_qf) - 2.0 * dm::s(m_omega) * m_n_int[0] * m_qf_dot);

    // Second vector
    arma::dvec fvf2 = m_g_mat.t() * (m_n_int[7] +
        2.0 * (m_n_int[8] + m_n_int[9]) * m_qf_dot -
        m_i_a_f * m_g_dot_mat * m_theta_dot);

    // Third vector
    arma::dvec fvf3 = (m_n_int[4] + m_n_int[5]).t() *
        m_g_dot_mat * m_theta_dot - m_n_int[10] - 2.0 * m_n_int[11] * m_qf_dot;

    // Centrifugal-coriolis force vector
    arma::dvec m_fvf = arma::join_vert(fvf1, fvf2, fvf3);
}


// N integrals calculation
void RayleighRitzBeam::n_integrals(void)
{

    /*************** Integral N1 ***************/
    arma::dmat phi_u_int = arma::zeros(1, m_axial_dofs);
    for (int i = 0; i < m_axial_dofs; i++)
    {
        double anl = m_alpha_freq(i) * m_beam_length;
        phi_u_int(i) = ((m_beam_density * m_beam_area * m_u_hat(i)) / 
            m_alpha_freq(i)) * (1.0 - cos(anl));
    }

    arma::dmat phi_v_int = arma::zeros(1, m_bending_y_dofs);
    for (int i = 0; i < m_bending_y_dofs; i++)
    {
        double bnl = m_beta_freq(i) * m_beam_length;
        phi_v_int(i) = ((m_beam_density * m_beam_area * m_v_hat(i)) / 
            m_beta_freq(i)) * (2.0 - cos(bnl) - cosh(bnl) + m_jvn(i) *
            (sin(bnl) - sinh(bnl)));
    }
    
    arma::dmat phi_w_int = arma::zeros(1, m_bending_z_dofs);
    for (int i = 0; i < m_bending_z_dofs; i++)
    {
        double gnl = m_gamma_freq(i) * m_beam_length;

        phi_w_int(i) = ((m_beam_density * m_beam_area * m_w_hat(i)) / 
            m_gamma_freq(i)) * (2.0 - cos(gnl) - cosh(gnl) + m_jwn(i) *
            (sin(gnl) - sinh(gnl)));
    }

    m_n_int[0] = arma::join_vert(phi_u_int * m_lu_mat, phi_v_int * m_lv_mat, 
        phi_w_int * m_lw_mat);

    /*************** Integral N2 ***************/
    m_n_int[1] = m_i_co_f + m_beam_mass * dm::s(m_raco_f_f).t() *
        dm::s(m_raco_f_f);

    /*************** Integral N3 ***************/
    // First row
    arma::dmat n3_row1 = arma::join_horiz((m_phi33_hat + m_phi22_hat) * m_qf,
        - m_phi21_hat * m_qf, - m_phi31_hat * m_qf);

    // Second row
    arma::dmat n3_row2 = arma::join_horiz(- m_phi12_hat * m_qf,
        (m_phi33_hat + m_phi11_hat) * m_qf, - m_phi32_hat * m_qf);

    // Third row
    arma::dmat n3_row3 = arma::join_horiz(- m_phi13_hat * m_qf,
        - m_phi23_hat * m_qf, (m_phi11_hat + m_phi22_hat) * m_qf);

    m_n_int[2] = arma::join_vert(n3_row1, n3_row2, n3_row3);

    /*************** Integral N4 ***************/
    // First row
    arma::dmat n4_row1 = arma::join_horiz(m_qf.t() * (m_phi33_dash +
        m_phi22_dash) * m_qf, - m_qf.t() * m_phi21_dash * m_qf,
        - m_qf.t() * m_phi31_dash * m_qf);

    // Second row
    arma::dmat n4_row2 = arma::join_horiz(- m_qf.t() * m_phi12_dash * m_qf,
        m_qf.t() * (m_phi33_dash + m_phi11_dash) * m_qf,
        - m_qf.t() * m_phi32_dash * m_qf);

    // Third row
    arma::dmat n4_row3 = arma::join_horiz(- m_qf.t() * m_phi13_dash * m_qf,
        - m_qf.t() * m_phi23_dash * m_qf,
        m_qf.t() * (m_phi22_dash + m_phi11_dash) * m_qf);

    m_n_int[3] = arma::join_vert(n4_row1, n4_row2, n4_row3);

    /*************** Inertia update ***************/
    // Inertial tensor of body wrt to point A (reference frame)
    m_i_a_f = m_n_int[1] + m_n_int[2] + m_n_int[2].t() + m_n_int[3];

    /*************** Integral N5 ***************/
    m_n_int[4] = arma::join_vert(m_phi32_hat - m_phi23_hat,
        m_phi13_hat - m_phi31_hat, m_phi21_hat - m_phi12_hat);

    /*************** Integral N6 ***************/
    m_n_int[5] = arma::join_vert(m_qf.t() * (m_phi32_dash - m_phi23_dash),
        m_qf.t() * (m_phi13_dash - m_phi31_dash),
        m_qf.t() * (m_phi21_dash - m_phi12_dash));

    /*************** Integral N7 ***************/
    m_n_int[6] = m_phi11_dash + m_phi22_dash + m_phi33_dash;

    /*************** Integral N8 ***************/
    m_n_int[7] = - dm::s(m_omega) * (m_i_a_f * m_omega);

    /*************** Integral N9 ***************/
    // First row
    arma::dmat n9_row1 = - m_omega(0) * (m_phi33_hat + m_phi22_hat) +
        m_omega(1) * m_phi21_hat + m_omega(2) * m_phi31_hat;

    // Second row
    arma::dmat n9_row2 = m_omega(0) * m_phi12_hat - m_omega(1) * 
        (m_phi11_hat + m_phi33_hat) + m_omega(2) * m_phi32_hat;

    // Third row
    arma::dmat n9_row3 = m_omega(0) * m_phi13_hat + m_omega(1) * m_phi23_hat -
        m_omega(2) * (m_phi11_hat + m_phi22_hat);

    m_n_int[8] = arma::join_vert(n9_row1, n9_row2, n9_row3);

    /*************** Integral N10 ***************/
    // First row
    arma::dmat n10_row1 = - m_qf.t() * m_omega(0) * (m_phi33_dash +
        m_phi22_dash) + m_qf.t() * m_omega(1) * m_phi21_dash +
        m_qf.t() * m_omega(2) * m_phi31_dash;

    // Second row
    arma::dmat n10_row2 = m_qf.t() * m_omega(0) * m_phi12_dash -
        m_qf.t() * m_omega(1) * (m_phi11_dash + m_phi33_dash) +
        m_qf.t() * m_omega(2) * m_phi32_dash;

    // Third row
    arma::dmat n10_row3 = m_qf.t() * m_omega(0) * m_phi13_dash +
        m_qf.t() * m_omega(1) * m_phi23_dash -
        m_qf.t() * m_omega(2) * (m_phi11_dash + m_phi22_dash);

    m_n_int[9] = arma::join_vert(n10_row1, n10_row2, n10_row3);
    
    /*************** Integral N11 ***************/
    m_n_int[10] = (m_n_int[8] + m_n_int[9]).t() * m_omega;

    /*************** Integral N11 ***************/
    m_n_int[11] = m_omega(0) * (m_phi32_dash - m_phi23_dash) + 
        m_omega(1) * (m_phi13_dash - m_phi31_dash) +
        m_omega(2) * (m_phi21_dash - m_phi12_dash);
}

/************ Shape integrals ************/
void RayleighRitzBeam::shape_integrals(void)
{
    /********* Shape integrals dash *********/
    //Integral phi11_dash
    m_phi11_dash = m_lu_mat.t() * m_lu_mat;
    
    //Integral phi12_dash
    arma::dmat phi_uv = arma::zeros(m_axial_dofs, m_bending_y_dofs);
    for(int k = 0; k < m_axial_dofs; k++)
    {
        for(int l = 0; l < m_bending_y_dofs; l++)
        {
            double ak = m_alpha_freq(k);
            double bl = m_beta_freq(l);
            double r1 = r_integrals(ak, bl, m_beam_length, 1);
            double r2 = r_integrals(ak, bl, m_beam_length, 2);
            double r3 = r_integrals(ak, bl, m_beam_length, 3);
            double r4 = r_integrals(ak, bl, m_beam_length, 4);

            double jvl = arma::as_scalar(m_jvn(l));
            phi_uv(k, l) = m_beam_density * m_beam_area * m_u_hat(k) *
                m_v_hat(l) * (r1 - r2 + jvl * r3 - jvl * r4);
        }
    }
    m_phi12_dash = m_lu_mat.t() * phi_uv * m_lv_mat;

    //Integral phi13_dash
    arma::dmat phi_uw = arma::zeros(m_axial_dofs, m_bending_z_dofs);

    for(int k = 0; k < m_axial_dofs; k++)
    {
        for(int l = 0; l < m_bending_z_dofs; l++)
        {
            double ak = m_alpha_freq(k);
            double gl = m_gamma_freq(l);
            double r1 = r_integrals(ak, gl, m_beam_length, 1);
            double r2 = r_integrals(ak, gl, m_beam_length, 2);
            double r3 = r_integrals(ak, gl, m_beam_length, 3);
            double r4 = r_integrals(ak, gl, m_beam_length, 4);

            double jwl = arma::as_scalar(m_jwn(l));

            phi_uw(k, l) = m_beam_density * m_beam_area * m_u_hat(k) *
                m_w_hat(l) * (r1 - r2 + jwl * r3 - jwl * r4);
        }
    }
    m_phi13_dash = m_lu_mat.t() * phi_uw * m_lw_mat;

    //Integral phi21_dash
    m_phi21_dash = m_phi12_dash.t();

    //Integral phi22_dash
    m_phi22_dash = m_lv_mat.t() * m_lv_mat;

    //Integral phi23_dash
    arma::dmat phi_vw = arma::zeros(m_bending_y_dofs, m_bending_z_dofs);

    for(int k = 0; k < m_bending_y_dofs; k++)
    {
        for(int l = 0; l < m_bending_z_dofs; l++)
        {
            double bk = m_beta_freq(k);
            double gl = m_gamma_freq(l);

            double r1 = r_integrals(bk, gl, m_beam_length, 1);
            double r2 = r_integrals(bk, gl, m_beam_length, 2);
            double r2_inv = r_integrals(gl, bk, m_beam_length, 2);
            double r3 = r_integrals(bk, gl, m_beam_length, 3);
            double r4 = r_integrals(bk, gl, m_beam_length, 4);
            double r4_inv = r_integrals(gl, bk, m_beam_length, 4);
            double r5 = r_integrals(bk, gl, m_beam_length, 5);
            double r6 = r_integrals(bk, gl, m_beam_length, 6);
            double r7 = r_integrals(bk, gl, m_beam_length, 7);
            double r8 = r_integrals(bk, gl, m_beam_length, 8);
            double r8_inv = r_integrals(gl, bk, m_beam_length, 8);
            double r9 = r_integrals(bk, gl, m_beam_length, 9);
            double r10 = r_integrals(bk, gl, m_beam_length, 10);
            double r10_inv = r_integrals(gl, bk, m_beam_length, 10);
            double r11 = r_integrals(bk, gl, m_beam_length, 11);

            double jvk = arma::as_scalar(m_jvn(k));
            double jwl = arma::as_scalar(m_jwn(l));

            phi_vw(k, l) = m_beam_density * m_beam_area * m_v_hat(k) *
                m_w_hat(l) * (r1 - r2 + jwl * r3 - jwl * r4 - r2_inv + r9 -
                jwl * r6 + jwl * r10 + jvk * r5 - jvk * r6 + jvk * jwl * r7 -
                jvk * jwl * r8 - jvk * r4_inv + jvk * r10_inv -
                jvk * jwl * r8_inv + jvk * jwl * r11);
        }
    }
    m_phi23_dash = m_lv_mat.t() * phi_vw * m_lw_mat;

    //Integral phi31_dash
    m_phi31_dash = m_phi13_dash.t();

    //Integral phi32_dash
    m_phi32_dash = m_phi23_dash.t();

    //Integral phi33_dash
    m_phi33_dash = m_lw_mat.t() * m_lw_mat;


    /********* Shape integrals hat *********/
    //Integral phi11_hat
    arma::drowvec phi_x1u = arma::zeros<arma::drowvec>(1, m_axial_dofs);

    for (int i = 0; i < m_axial_dofs; i++)
    {
        double anl = m_alpha_freq(i) * m_beam_length;
        phi_x1u(i) = (m_u_freq(i) * m_beam_density * pow(m_beam_radius, 2.0) *
            M_PI / pow(m_alpha_freq(i), 2.0)) * (sin(anl) - anl * cos(anl));
    }
    m_phi11_hat = phi_x1u * m_lu_mat;

    //Integral phi12_hat
    arma::drowvec phi_x1v = arma::zeros<arma::drowvec>(1, m_bending_y_dofs);

    for (int i = 0; i < m_bending_y_dofs; i++)
    { 
        double bnl = m_beta_freq(i) * m_beam_length;
        phi_x1v(i) = (m_v_freq(i) * m_beam_density * pow(m_beam_radius, 2.0) * 
            M_PI / pow(m_beta_freq(i), 2.0)) * (sin(bnl) - bnl * cos(bnl) - 
            bnl * cosh(bnl) + sinh(bnl) + m_jvn(i) * (bnl * sin(bnl) +
            cos(bnl) - bnl * sinh(bnl) + cosh(bnl) - 2.0));
    }
    m_phi12_hat = phi_x1v * m_lv_mat;

    //Integral phi13_hat
    arma::drowvec phi_x1w = arma::zeros<arma::drowvec>(1, m_bending_z_dofs);

    for (int i = 0; i < m_bending_z_dofs; i++)
    { 
        double gnl = m_gamma_freq(i) * m_beam_length;
        phi_x1w(i) = (m_w_freq(i) * m_beam_density * pow(m_beam_radius, 2.0) * 
            M_PI / pow(m_gamma_freq(i), 2.0)) * (sin(gnl) - gnl * cos(gnl) - 
            gnl * cosh(gnl) + sinh(gnl) + m_jwn(i) * (gnl * sin(gnl) +
            cos(gnl) - gnl * sinh(gnl) + cosh(gnl) - 2.0));
    }
    m_phi13_hat = phi_x1w * m_lw_mat;

    //Integral phi21_hat
    arma::drowvec phi_x2u = arma::zeros<arma::drowvec>(1, m_axial_dofs);
    m_phi21_hat = phi_x2u * m_lu_mat;

    //Integral phi22_hat
    arma::drowvec phi_x2v = arma::zeros<arma::drowvec>(1, m_bending_y_dofs);
    m_phi22_hat = phi_x2v * m_lv_mat;

    //Integral phi23_hat
    arma::drowvec phi_x2w = arma::zeros<arma::drowvec>(1, m_bending_z_dofs);
    m_phi23_hat = phi_x2w * m_lw_mat;

    //Integral phi31_hat
    arma::drowvec phi_x3u = arma::zeros<arma::drowvec>(1, m_axial_dofs);
    m_phi31_hat = phi_x3u * m_lu_mat;

    //Integral phi32_hat
    arma::drowvec phi_x3v = arma::zeros<arma::drowvec>(1, m_bending_y_dofs);
    m_phi32_hat = phi_x3v * m_lv_mat;

    //Integral phi23_hat
    arma::drowvec phi_x3w = arma::zeros<arma::drowvec>(1, m_bending_z_dofs);
    m_phi33_hat = phi_x3w * m_lw_mat;

}


/**************** R Integrals ****************/
double RayleighRitzBeam::r_integrals(double a, double b, double l, int id)
{
    double r = 0.0; double c_m = a - b; double c_p = a + b; 
    double c_ms = pow(a, 2.0) - pow(b, 2.0); double c_ps = pow(a, 2.0) + pow(b, 2.0); 
    double tol = 1e-5;
    bool a_b_equal = abs(a - b < tol);

    switch (id)
    {
    case 1:
        if (a_b_equal)
        {
            r = (2.0 * a * l - sin(2.0 * a * l)) / (4.0 * a);
        }
        else
        {
            r = sin(c_m * l) / (2.0 * c_m) - sin(c_p * l) / (2.0 * c_p);
        }
        break; 

    case 2:
        r = (1.0 / c_ps) * (b * cosh(b * l) * sin(a * l) - 
            a * cos(a * l) * sinh(b * l));
        break;

    case 3:
        if (a_b_equal)
        {
            r = pow(sin(a * l), 2.0) / (2.0 * a);
        } 
        else
        {
            r = (1.0 - cos(c_p * l)) / (2.0 * c_p) +
                (1.0 - cos(c_m * l)) / (2.0 * c_m);
        }
        break;

    case 4:
        r = (1.0 / c_ps) * (b * sin(a * l) * sinh(b * l) -
            a * (cos(a * l) * cosh(b * l) - 1.0));
        break;

    case 5:
        if (a_b_equal)
        {
            r = pow(sin(a * l), 2.0) / (2.0 * a);
        }
        else
        {
            r = (cos(c_m * l) - 1.0) / (2.0 * c_m) +
                (1.0 - cos(c_p * l)) / (2.0 * c_p);
        }
        break; 

    case 6:
        r = (1.0 / c_ps) * (b * (cos(a * l) * cosh(b * l) - 1.0) + 
            a * sin(a * l) * sinh(b * l));
        break;

    case 7:
        if (a_b_equal)
        {
            r = (2.0 * a * l + sin(2.0 * a * l)) / (4.0 * a);
        }
        else
        {
            r = sin(c_m * l) / (2.0 * c_m) + sin(c_p * l) / (2.0 * c_p);
        }
        break;

    case 8:
        r = (1.0 / c_ps) * (a * sin(a * l) * cosh(b * l) + 
            b * cos(a * l) * sinh(b * l));
        break;

    case 9:
        if (a_b_equal)
        {
            r = (sinh(2.0 * a * l) - 2.0 * a * l) / (4.0 * a);
        }
        else
        {
            r = (1.0 / c_ms) * (a * cosh(a * l) * sinh(b * l) -
                b * cosh(b * l) * sinh(a * l));
        }
        break; 

    case 10:
        if (a_b_equal)
        {
            r = pow(sinh(a * l), 2.0) / (2.0 * a);
        }
        else
        {
            r = (1.0 / c_ms) * (a * (cosh(a * l) * cosh(b * l) - 1.0) -
                b * sinh(a * l) * sinh(b * l));
        }
        break;

    case 11:
        if (a_b_equal)
        {
            r = (sinh(2.0 * a * l) + 2.0 * a * l) / (4.0 * a);
        }
        else
        {
            r = (1.0 / c_ms) * (a * cosh(b * l) * sinh(a * l) -
                b * cosh(a * l) * sinh(b * l));
        }
        break;

    default:
        r = 0.0;
        break;
    }

    return r;
}