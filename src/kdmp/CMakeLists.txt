file(GLOB_RECURSE KDMP_SOURCE src/*.cpp)


find_package(ompl REQUIRED)

include_directories(
    include
    ${OMPL_INCLUDE_DIRS}
)

add_library( kdmp 
    ${KDMP_SOURCE}
)
target_link_libraries(kdmp ompl)
# install (TARGETS kdmp 
#         LIBRARY DESTINATION lib)