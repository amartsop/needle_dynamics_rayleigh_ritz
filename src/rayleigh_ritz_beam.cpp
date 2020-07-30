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

    // Initialization of frequencies vectors 
    m_u_freq = arma::zeros(m_axial_dofs);
    m_v_freq = arma::zeros(m_bending_y_dofs);
    m_w_freq = arma::zeros(m_bending_z_dofs);

    // // Calculate mass matrix
    // mass_matrix_calculation();
    
    // // Calculate stiffness matrix 
    // stiffness_matrix_calculation();
}




/**************** R Integrals ****************/
double m_r_integrals(double a, double b, double l, int int_id)
{
    double r = 0; double c_m = a - b; double c_p = a + b; 
    double c_ms = pow(a, 2.0) - pow(b, 2.0); double c_ps = pow(a, 2.0) + pow(b, 2.0); 

    switch (int_id)
    {
    case 1:
        r = sin(c_m * l) / (2.0 * c_m) - sin(c_p * l) / (2.0 * c_p);
        break;
    case 2:
        r = (1.0 / c_ps) * (b * cosh(b * l) * sin(a * l) - 
            a * cos(a * l) * sinh(a * l));
        break;
    case 3:
        r = (1.0 - cos(c_p * l)) / (2.0 * c_p) +
            (1.0 - cos(c_m * l)) / (2.0 * c_m);
        break;
    case 4:
        r = (1.0 / c_ps) * (b * sin(a * l) * sinh(b * l) -
            a * (cos(a * l) * cosh(b * l) - 1.0));
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:
        break;
    case 8:
        break;
    case 9:
        break;
    case 10:
        break;
    case 11:
        break;
    default:
        r = 0;
        break;
    }




    
    double r5 = (cos(c_m * l) - 1.0) / (2.0 * c_m) +
        (1.0 - cos(c_p * l)) / (2.0 * c_p);

    double r6 = (1.0 / c_ps) * (b * (cos(a * l) * cosh(b * l) - 1.0) + 
        a * sin(a * l) * sinh(b * l));

    double r7 = sin(c_m * l) / (2.0 * c_m) + sin(c_p * l) / (2.0 * c_p);

    double r8 = (1.0 / c_ps) * (a * sin(a * l) * cosh(b * l) + 
        b * cos(a * l) * sinh(b * l));

    double r9 = (1.0 / c_ms) * (a * cosh(a * l) * sinh(b * l) -
        b * cosh(b * l) * sinh(a * l));

    double r10 = (1.0 / c_ms) * (a * (cosh(a * l) * cosh(b * l) - 1.0) -
        b * sinh(a * l) * sinh(b * l));

    double r11 = (1.0 / c_ms) * (a * cosh(b * l) * sinh(a * l) -
        b * cosh(a * l) * sinh(b * l));
}