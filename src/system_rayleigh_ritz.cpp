#include "system_rayleigh_ritz.h"


SystemRayleighRitz::SystemRayleighRitz(Handle *handle, RayleighRitzBeam *needle,
    InputTrajectory *input_traj)
{
    // Rigid body (handle)
    m_handle_ptr = handle;

    // Flexible body (needle)
    m_needle_ptr = needle;

    // Input coordinates 
    m_input_traj_ptr = input_traj;

    // Number of dofs
    m_dofs = needle->get_elastic_dofs();
}


arma::dvec SystemRayleighRitz::calculate( arma::dvec state_vector, double t)
{
    // Update rigid body trajectory
    m_input_traj_ptr->update(t);

    /***************** Rigid body state ***********************/
    // Displacement
    arma::dvec roc_F_F = m_input_traj_ptr->get_linear_displacement();
    arma::dvec theta_r = m_input_traj_ptr->get_rotational_displacement();
    arma::dvec qr = arma::join_vert(roc_F_F, theta_r);

    // Velocity
    arma::dvec roc_dot_F_F = m_input_traj_ptr->get_linear_velocity();
    arma::dvec theta_dot_r = m_input_traj_ptr->get_rotational_velocity();
    arma::dvec qr_dot = arma::join_vert(roc_dot_F_F, theta_dot_r);

    // Acceleration
    arma::dvec roc_ddot_F_F = m_input_traj_ptr->get_linear_acceleration();
    arma::dvec theta_ddot_r = m_input_traj_ptr->get_rotational_acceleration();
    arma::dvec qr_ddot = arma::join_vert(roc_ddot_F_F, theta_ddot_r);

    // Update rigid body matrices 
    m_handle_ptr->update(t, qr, qr_dot);

    // Rotation matrix
    arma::dmat rot_f_F = m_handle_ptr->get_rotation_matrix();

    // G and g_dot matrix matrix 
    arma::dmat g_mat = m_handle_ptr->get_g_matrix();
    arma::dmat g_dot_mat = m_handle_ptr->get_g_dot_matrix();

    // Distance between frames 
    arma::dvec rca_f_f = m_handle_ptr->get_rca_f_f();

    // Omega 
    arma::dvec omega = g_mat * theta_dot_r;

    /***************** Flexible body state ***********************/
    // Displacement
    arma::dvec roa_F_F = roc_F_F + rot_f_F * rca_f_f;
    arma::dvec theta_f = theta_r;
    arma::dvec qf = state_vector.rows(0, m_dofs - 1);
    arma::dvec q = arma::join_vert(roa_F_F, theta_f, qf);

    // Velocity
    arma::dvec roa_dot_F_F = roc_dot_F_F - rot_f_F * dm::s(rca_f_f) * omega;
    arma::dvec theta_dot_f = theta_dot_r;
    arma::dvec qf_dot = state_vector.rows(m_dofs, 2 * m_dofs - 1);
    arma::dvec q_dot = arma::join_vert(roa_dot_F_F, theta_dot_f, qf_dot);

    // Update flexible body matrices 
    m_needle_ptr->update(t, q, q_dot);

    // Get mass matrices
    arma::dmat mf31 = m_needle_ptr->get_mf31_matrix();
    arma::dmat mf32 = m_needle_ptr->get_mf32_matrix();
    arma::dmat mf33 = m_needle_ptr->get_mf33_matrix();

    // Get damping and stiffness matrices 
    arma::dmat cf33 = m_needle_ptr->get_cf33_matrix();
    arma::dmat kf33 = m_needle_ptr->get_kf33_matrix();

    // Get fvf3 vector
    arma::dvec fvf3 = m_needle_ptr->get_fvf3_vector();

    // Get qf3 vector
    arma::dvec qf3 = m_needle_ptr->get_qf3_vector();

    // Acceleration
    arma::dvec roa_ddot_F_F = roc_ddot_F_F + rot_f_F * (dm::s(omega) + 
        dm::s(omega) * dm::s(omega)) * rca_f_f;
    arma::dvec theta_ddot_f = theta_ddot_r;

    // Calculate tau3
    arma::dvec tau3 = fvf3 + qf3 - mf31 * roa_ddot_F_F - mf32 * theta_ddot_f;

    // Calculate elastic acceleration
    arma::dvec qf_ddot = arma::solve(mf33, tau3 - cf33 * qf_dot - kf33 * qf);

    // Total Acceleration
    arma::dvec q_ddot = arma::join_vert(roa_ddot_F_F, theta_ddot_f, qf_ddot);

    // Reaction forces 
    arma::dmat mf = m_needle_ptr->get_mass_matrix();
    arma::dmat cf = m_needle_ptr->get_damping_matrix();
    arma::dmat kf = m_needle_ptr->get_stiffness_matrix();
    arma::dvec fvf = m_needle_ptr->get_coriolis_vector();
    arma::dvec qforce = m_needle_ptr->get_external_force();

    // Total reaction force
    arma::dvec reaction_force = mf * q_ddot + cf * q_dot + kf * q - fvf - qforce;

    // Reaction force at point a 
    arma::dvec qc1 = {reaction_force(0), reaction_force(1), reaction_force(2)};
    arma::dvec qc2 = {reaction_force(3), reaction_force(4), reaction_force(5)};
    arma::dvec fa_F = qc1; arma::dvec ma_f = arma::solve(g_mat.t(), qc2);

    // Reaction force at point c
    arma::dvec fc_F = m_handle_ptr->get_handle_mass() * roc_ddot_F_F -
        m_handle_ptr->get_weight_force() - fa_F;

    m_fc_f = rot_f_F.t() * fc_F;

    // Handle inertial tensor 
    arma::dmat ic_f =m_handle_ptr->get_inertial_tensor(); 

    // Reaction moment at point c
    m_mc_f = ic_f * (g_dot_mat * theta_r + g_mat * theta_ddot_r) - ma_f -
        dm::s(m_handle_ptr->get_rca_f_f()) * rot_f_F.t() * fa_F -
        dm::s(omega) * ic_f * omega;


    return arma::join_vert(qf_dot, qf_ddot);
}


