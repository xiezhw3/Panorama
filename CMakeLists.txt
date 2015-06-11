project (imageSitich)

add_definitions(-std=c++11)

file(GLOB_RECURSE imageSitich_SOURCES "src/*.cpp")

set (imageSitich_INCLUDE_DIRS "")
foreach (_headerFile ${imageSitich_HEADERS})
    get_filename_component(_dir ${_headerFile} PATH)
    list (APPEND imageSitich_INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES imageSitich_INCLUDE_DIRS)

include_directories(${imageSitich_INCLUDE_DIRS})
find_package( OpenCV REQUIRED )
add_executable (imageSitichExe ${imageSitich_SOURCES})
target_link_libraries( imageSitichExe ${OpenCV_LIBS} )