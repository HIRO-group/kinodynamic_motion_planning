add_subdirectory(kdmp)
install(DIRECTORY kdmp/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/kdmp"
    COMPONENT kdmp
    FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)
