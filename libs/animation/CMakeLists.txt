add_subdirectory(libs)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configuration/root_directory.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/include/root_directory.h)

add_library(animation STATIC
	./src/filesystem.cpp
	./src/display.cpp
	./src/stb_image.cpp
	./src/shader.cpp
	./src/texture.cpp
	./src/camera.cpp
	./src/callback_handler.cpp
	./src/callback_setter.cpp
	./src/transform.cpp
	./src/obj_loader.cpp
	./src/mesh.cpp
	./src/indexed_model.cpp
	./src/animation.cpp
)

target_link_libraries(animation PRIVATE graphics)


target_include_directories(animation PUBLIC include libs)






