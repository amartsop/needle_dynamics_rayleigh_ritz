if(MSVC AND NOT "${MSVC_VERSION}" LESS 1400)
	add_definitions( "/MP" )
endif()

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

add_library(graphics INTERFACE)

# Include glfw library
add_subdirectory(glfw-3.1.2)

#Link OPENGL library
cmake_policy(SET "CMP0072" OLD)
find_package(OpenGL REQUIRED)


if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
set(OPENGL_LIBRARY_FULL
	${OPENGL_LIBRARY}
	-lGLU -lXrandr -lXext -lX11 -lrt
	${CMAKE_DL_LIBS}
	${GLFW_LIBRARIES}
)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
set(OPENGL_LIBRARY_FULL
	${OPENGL_LIBRARY}
	${CMAKE_DL_LIBS}
	${GLFW_LIBRARIES}
)
endif()

# Create GLAD library
add_library(GLAD STATIC
	glad/glad.h
	glad/glad.c)

target_link_libraries(GLAD PRIVATE ${OPENGL_LIBRARY_FULL})
target_include_directories(GLAD PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


# Create GLEW library
add_library( GLEW_1130 STATIC
	glew-1.13.0/src/glew.c
)

target_link_libraries(graphics INTERFACE ${OPENGL_LIBRARY_FULL} glfw GLAD GLEW_1130)
