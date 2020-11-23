#include "camera.h"


Camera::Camera(GLFWwindow* window, const glm::vec3& pos, float fov,
    float aspect, float zNear, float zFar)
{
    // Window handler 
    m_window = window;

    // Initialize projection
    m_projection = glm::perspective(glm::radians(fov), aspect, zNear, zFar);

    // Initialise camera axes
    m_cameraFront = m_axisZ;
    m_cameraUp = m_axisY;

    // Initialize camera position
    m_position = pos;

    // Initialize aspect and camera distances
    m_aspect = aspect; m_zNear = zNear; m_zFar = zFar; 
}


void Camera::update(double deltaTime)
{
    // Process input
    processKeyboardInput(deltaTime, m_cameraFront);

    // Camera projection 
    m_projection = glm::perspective(glm::radians(m_fov), m_aspect, m_zNear, m_zFar);

    // Aling camera frame to object desired frame
    glm::mat4 rot1 = glm::rotate(glm::radians(90.0f), m_axisY);
    glm::mat4 rot2 = glm::rotate(glm::radians(90.0f), m_axisZ);
    glm::mat4 rot3 = glm::rotate(glm::radians(-m_yaw), m_axisY);
    glm::mat4 rot4 = glm::rotate(glm::radians(-m_pitch), m_axisX);

    glm::mat4 rot = rot1 * rot2 * rot3 * rot4;
    glm::vec3 cameraFront = rot * glm::vec4(m_axisZ, 1.0f);
    glm::vec3 cameraUp = rot * glm::vec4(m_axisY, 1.0f);

    // Update camera front and up axis
    m_cameraFront = glm::vec3(cameraFront);
    m_cameraUp = glm::vec3(cameraUp);


    // Camera view
    m_view = glm::lookAt(m_position, m_position + m_cameraFront, m_cameraUp);

    std::cout << m_position.x << ", " << m_position.y << ", " << m_position.z << std::endl;
    std::cout << m_pitch << std::endl;
    std::cout << m_yaw << std::endl;
}


void Camera::processMouseInput(float xCursorOffset, float yCursorOffset)
{
    m_yaw += xCursorOffset; m_pitch += yCursorOffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (m_pitch > 89.0f) m_pitch = 89.0f;
    if (m_pitch < -89.0f) m_pitch = -89.0f;
}


void Camera::processKeyboardInput(double deltaTime, const glm::vec3& cameraFront)
{
        
    float cameraSpeed = m_cameraSpeedScale * (float)deltaTime;

    if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
    {
        m_position += cameraSpeed * cameraFront;
    }

    if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS)
    {
        m_position -= cameraSpeed * cameraFront;
    }

    if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS)
    {
        m_position -= glm::normalize(glm::cross(cameraFront, m_cameraUp)) *
            cameraSpeed;
    }

    if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS)
    {
        m_position += glm::normalize(glm::cross(cameraFront, m_cameraUp)) *
            cameraSpeed;
    }
}

void Camera::processMouseWheel(float yWheel)
{
    m_fov -= yWheel;

    if (m_fov < 1.0f) m_fov = 1.0f;
    if (m_fov > 45.0f) m_fov = 45.0f;
}