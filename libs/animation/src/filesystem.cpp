#include <filesystem.h>
#include <root_directory.h>


std::string FileSystem::getAbsolutePath(void)
{
    static char const * envRoot = getenv("LOGL_ROOT_PATH");
    static char const * givenRoot = (envRoot != nullptr ? envRoot : logl_root);
    static std::string root  = (givenRoot != nullptr ? givenRoot : "");
    return root;
}

