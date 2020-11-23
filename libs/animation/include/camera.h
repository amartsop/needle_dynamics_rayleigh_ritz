#pragma once

#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <GLFW/glfw3.h>



class Camera
{
    public:
        //Camera constructor
        Camera(GLFWwindow* window, const glm::vec3& pos, float fov,
            float aspect, float zNear, float zFar);

        void update(double deltaTime);

        glm::mat4 getProjection(void) const { return m_projection; }
        glm::mat4 getView(void) const { return m_view; }
        glm::vec3 getCameraPosition(void) const { return m_position; }

        void processMouseInput(float xCursorOffset, float yCursorOffset);

        void processMouseWheel(float yWheel);

    private:
        GLFWwindow* m_window;

        void processKeyboardInput(double deltaTime, const glm::vec3& cameraFront);

        // World axes
        const glm::vec3 m_axisX = glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 m_axisY = glm::vec3(0.0f, 1.0f, 0.0f);
        const glm::vec3 m_axisZ = glm::vec3(0.0f, 0.0f, 1.0f);
        
        // Camera axes
        glm::vec3 m_cameraUp, m_cameraFront, m_cameraRight;

        // Camera speed scale
        const float m_cameraSpeedScale = 0.5f;

        // Camera aspect and depth parameres 
        float m_aspect, m_zNear, m_zFar;

        // Camera view and projection
        glm::mat4 m_projection, m_view;

        // Camera position 
        glm::vec3 m_position;

        // Camera pitch and yaw
        float m_pitch =  12.0f; float m_yaw = 145.0f;


        // Camera field of view
        float m_fov = 45.0f;
};