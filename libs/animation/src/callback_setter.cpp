#include <callback_setter.h>


CallbackSetter::CallbackSetter(GLFWwindow* window)
{
    // Mouse cursor position callback 
    glfwSetCursorPosCallback(window, mouseCursorCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Mouse wheel scroll callback
    glfwSetScrollCallback(window, mouseWheelCallback);
}


void CallbackSetter::mouseCursorCallback(GLFWwindow* window, double xpos,
    double ypos)
{
    CallbackHandler *event  = 
        reinterpret_cast<CallbackHandler *>(glfwGetWindowUserPointer(window));

    if(event)
    {
        event->mouseCursorCallback((float)xpos, (float)ypos);
    }
}


void CallbackSetter::mouseWheelCallback(GLFWwindow* window, double xoffset,
    double yoffset)
{
    CallbackHandler *event  = 
        reinterpret_cast<CallbackHandler *>(glfwGetWindowUserPointer(window));

    if(event)
    {
        event->mouseWheelCallback((float)xoffset, (float)yoffset);
    }
}