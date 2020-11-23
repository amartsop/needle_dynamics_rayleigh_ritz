#pragma once 

#include <iostream>
#include <fstream>

#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <filesystem.h>
#include <display.h>
#include <callback_setter.h>
#include <callback_handler.h>
#include <mesh.h>
#include <shader.h>
#include <transform.h>
#include <texture.h>



class Animation
{
    
public:
    Animation();

    bool isDone(void) { return m_display.isClosed(); }
    void update(double deltaTime);

    // Set handle position 
    void setHandlePosition(const glm::vec3& pos) { m_handlePosition = pos; }

    // Set handle orientation
    void setHandleOrientation(const glm::vec3& rot) { m_handleOrientation = rot; }


    // Needle mesh information
    std::vector<glm::vec3> getNeedleVerticesPos(void) const;
    std::vector<glm::vec2> getNeedleTextureCoords(void) const;
    std::vector<glm::vec3> getNeedleNormals(void) const;
    unsigned int getNeedleVerticesNum(void) const;

    // Update mesh
    void setNeedleVerticesPos(const std::vector<glm::vec3>& vertPos);

private:
    // Update lighting and camera setup
    void updateSetup(void);

    // Update and draw needle
    void drawHandle(void);

    // Update and draw handle
    void drawNeedle(void);

private:
    glm::vec3 m_handlePosition = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 m_handleOrientation = glm::vec3(0.0f, 0.0f, 0.0f);

private: 
   const glm::vec3 handleOffset = glm::vec3(0.0f, 0.0f, 0.0f);

    // Needle geometry
    const float m_needle_lx = 7.65e-2; // Needle offset x direction (m)
    const float m_needle_ly = 0.0; // Needle offset x direction (m)
    const float m_needle_lz = -7.67e-3; // Needle offset z direction (m)

    const glm::vec3 m_needleOffset = glm::vec3(m_needle_lx, m_needle_ly,
        m_needle_lz);

private:
    /*********************** Filepath ***********************/
    // FileSystem m_filepath = FileSystem();
    std::string absolutePath = FileSystem::getAbsolutePath();

    /*********************** Display ***********************/
    // Window properties
    const char* windowTitle = "Transperineal Prostate Biopsy";
    const float backgroundColor[3] = {0.3f, 0.3f, 0.3f}; // RGB

    // Display object
    Display m_display = Display(windowTitle);
        
    // Window handle
    GLFWwindow* m_window = m_display.getWindowHandle();

    /******************* Set Callback Functions *******************/
    CallbackSetter m_callbackSet = CallbackSetter(m_window);
    CallbackHandler m_callbackHandler = CallbackHandler(m_window);

    /*********************** Camera ***********************/
    // Camera constants
    const glm::vec3 cameraPos = glm::vec3(0.44f, 0.14f, 0.012f);
    const float cameraFOV = 45.0f; const float zNear = 0.01f;
    const float zFar = 1000.0f;

    // Camera object
    Camera m_camera = Camera(m_window, cameraPos, cameraFOV,
        m_display.getWindowWidth() / (float)m_display.getWindowHeight(),
        zNear, zFar);

    /*********************** Mesh ***********************/
    // Mesh constants
    const std::string handleMeshName = absolutePath + "/objects/handle/handle.obj";
    const std::string needleMeshName = absolutePath + "/objects/needle/needle.obj";

    // Mesh object
    Mesh m_meshHandle = Mesh(handleMeshName, GL_DYNAMIC_DRAW);
    Mesh m_meshNeedle = Mesh(needleMeshName, GL_DYNAMIC_DRAW);

    /*********************** Shader ***********************/
    // Shader constants
    const std::string objectShaderName = absolutePath + "/share/simpleShader";
    const std::string lighttShaderName = absolutePath + "/share/lightShader";

    // Shader object
    Shader m_objectShader = Shader(objectShaderName);
    Shader m_lightShader = Shader(lighttShaderName);

    /*********************** Transformation ***********************/
    Transform m_handleTransform, m_needleTransform, m_lightTransform;

    /*********************** Texture ***********************/
    // Texture constants
    const std::string handleTextureName = absolutePath + "/objects/handle/handle.png";
    const std::string needleTextureName = absolutePath + "/objects/needle/needle.png";

    // Texture objects
    Texture m_handleTexture = Texture(handleTextureName);
    Texture m_needleTexture = Texture(needleTextureName);

    /*********************** Lighting ***********************/
    glm::vec3 m_lightPos = glm::vec3(1.0f, 1.0f, 1.0f);
};

