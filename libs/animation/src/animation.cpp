#include "../animation.h"

Animation::Animation()
{
    /******************* Set Callbacks *******************/
    // Set cursor callback
    m_callbackHandler.setCursorCoords((float)m_display.getWindowWidth() / 2.0f, 
        (float)m_display.getWindowHeight() / 2.0f);

    // Set camera callback inputs
    m_callbackHandler.setCamera(&m_camera);
}


void Animation::update(double deltaTime)
{
    /**************** Display ****************/
    // Clear display background
    m_display.clear(backgroundColor[0], backgroundColor[1], backgroundColor[2]);

    /**************** Camera ****************/
    // Update camera 
    m_camera.update(deltaTime);

    /**************** Shader bind and setup update ****************/
    // Bind shader 
    m_objectShader.bind();

    // Update setup
    updateSetup();

    /*********************** Draw Geometries ***********************/
    // Update handle
    drawHandle();
    
    // Update needle
    drawNeedle();

    // Poll events
    glfwPollEvents();
}


// Update and draw handle
void Animation::drawHandle()
{
    // Handle transform 
    m_handleTransform.setPos(m_handlePosition);
    m_handleTransform.setRot(m_handleOrientation);

    // Draw handle
    glm::mat4 modelHandle = m_handleTransform.getModel();
    m_objectShader.setMat4("model", modelHandle);

    // Bind texture 
    m_handleTexture.bind(0);

    // Draw mesh
    m_meshHandle.draw();
}

// Update and draw needle
void Animation::drawNeedle(void)
{
    // Needle transform 
    m_needleTransform.setRot(glm::vec3(0.0f, 0.0f, 0.0f));
    m_needleTransform.setPos(m_needleOffset + glm::vec3(0.0f, 0.0f, 0.0f));

    // Draw Needle
    glm::mat4 modelNeedle = m_needleTransform.getModel();
    m_objectShader.setMat4("model", modelNeedle);

    // Bind texture 
    m_needleTexture.bind(0);

    // Draw mesh
    m_meshNeedle.draw();

    // Update display
    m_display.update();
}

// Update lighting and camera setup
void Animation::updateSetup(void)
{

    // Camera and light position
    m_objectShader.setVec3("viewPos", m_camera.getCameraPosition());
    m_objectShader.setVec3("light.position", m_lightPos);

    // Light properties
    m_objectShader.setVec3("light.ambient", 0.2f, 0.2f, 0.2f); 
    m_objectShader.setVec3("light.diffuse", 0.5f, 0.5f, 0.5f);
    m_objectShader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);

    // Material properties
    m_objectShader.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
    m_objectShader.setFloat("material.shininess", 64.0f);

    // View and projection
    m_objectShader.setMat4("projection", m_camera.getProjection());
    m_objectShader.setMat4("view", m_camera.getView());
}


// Get the position of needle's vertices
std::vector<glm::vec3> Animation::getNeedleVerticesPos(void) const
{
    return m_meshNeedle.getObjVerticesPos();
}

// Get needle's texture coords
std::vector<glm::vec2> Animation::getNeedleTextureCoords(void) const
{
    return m_meshNeedle.getObjTextureCoords();
}


std::vector<glm::vec3> Animation::getNeedleNormals(void) const
{
    return m_meshNeedle.getObjNormals();
}


unsigned int Animation::getNeedleVerticesNum(void) const
{
    return m_meshNeedle.getObjVerticesNum();
}


void Animation::setNeedleVerticesPos(const std::vector<glm::vec3>& vertPos)
{
    m_meshNeedle.setObjVerticesPos(vertPos);
}