add_subdirectory(kdmp)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY kdmp/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/kdmp"
    COMPONENT kdmp
    FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN "*.cpp"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
