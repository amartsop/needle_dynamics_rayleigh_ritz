#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include <callback_handler.h>

class CallbackSetter
{
    public:
        CallbackSetter(GLFWwindow* window);

    private:
        // Cursor position calllback function
        static void mouseCursorCallback(GLFWwindow* window, double xpos,
            double ypos);

        // Scroll wheel callback function
        static void mouseWheelCallback(GLFWwindow* window, double xoffset,
            double yoffset);
};


