file(GLOB_RECURSE KDMP_SOURCE_CODE models/*.cpp spaces/*.cpp problems/*.cpp planners/*.cpp)
file(GLOB_RECURSE KDMP_HEADERS models/*.h models/*.hpp spaces/*.hpp base/*.hpp problems/*.hpp planners/*.hpp)
file(GLOB_RECURSE KDMP_SOURCE src/*.cpp)


include_directories(
    include
)

add_library( kdmp 
    ${KDMP_SOURCE}
)
# install (TARGETS kdmp 
#         LIBRARY DESTINATION lib)