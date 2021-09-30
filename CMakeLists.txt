# https://gamephysicsweekend.github.io/
cmake_minimum_required (VERSION 3.0)
project( gpiow )

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(cmake/GitUtils.cmake)

git_clone(
        PROJECT_NAME    openGJK
        GIT_URL         https://github.com/MattiaMontanari/openGJK
        GIT_BRANCH      master
)

set(opengjk_src
	${CMAKE_BINARY_DIR}/openGJK/src/openGJK.c
	${CMAKE_BINARY_DIR}/openGJK/include/openGJK/openGJK.h
)

git_clone(
        PROJECT_NAME    libccd
        GIT_URL         https://github.com/danfis/libccd
        GIT_BRANCH      master
)

set(CCD_DOUBLE OFF)
set(CCD_SINGLE ON)
configure_file(${CMAKE_BINARY_DIR}/libccd/src/ccd/config.h.cmake.in ${CMAKE_BINARY_DIR}/libccd/src/ccd/config.h)

set(libccd_src
  ${CMAKE_BINARY_DIR}/libccd/src/ccd/ccd.h
  ${CMAKE_BINARY_DIR}/libccd/src/ccd/compiler.h
  ${CMAKE_BINARY_DIR}/libccd/src/ccd/ccd_export.h
  ${CMAKE_BINARY_DIR}/libccd/src/ccd/quat.h
  ${CMAKE_BINARY_DIR}/libccd/src/ccd/vec3.h
  ${CMAKE_BINARY_DIR}/libccd/src/ccd/config.h

  ${CMAKE_BINARY_DIR}/libccd/src/alloc.h
  ${CMAKE_BINARY_DIR}/libccd/src/simplex.h
  ${CMAKE_BINARY_DIR}/libccd/src/polytope.h
  ${CMAKE_BINARY_DIR}/libccd/src/support.h
  ${CMAKE_BINARY_DIR}/libccd/src/dbg.h
  ${CMAKE_BINARY_DIR}/libccd/src/list.h

  ${CMAKE_BINARY_DIR}/libccd/src/ccd.c
  ${CMAKE_BINARY_DIR}/libccd/src/mpr.c
  ${CMAKE_BINARY_DIR}/libccd/src/polytope.c
  ${CMAKE_BINARY_DIR}/libccd/src/support.c
  ${CMAKE_BINARY_DIR}/libccd/src/vec3.c
)

set(gpiow_src_book00
	./gpiow/Scene_Book00/Scene.h
	./gpiow/Scene_Book00/Scene.cpp
)

set(gpiow_src_book01
	./gpiow/Scene_Book01/Scene.h
	./gpiow/Scene_Book01/Scene.cpp
)

set(gpiow_physics_src
	./gpiow/Physics/Body.cpp
	./gpiow/Physics/Body.h
	./gpiow/Physics/Broadphase.cpp
	./gpiow/Physics/Broadphase.h
	./gpiow/Physics/Contact.cpp
	./gpiow/Physics/Contact.h
	./gpiow/Physics/GJK.cpp
	./gpiow/Physics/GJK.h
	./gpiow/Physics/Intersections.cpp
	./gpiow/Physics/Intersections.h
	./gpiow/Physics/Shapes.cpp
	./gpiow/Physics/Shapes.h
)

set(gpiow_physics_shapes_src
	./gpiow/Physics/Shapes/ShapeBase.h
	./gpiow/Physics/Shapes/ShapeBox.cpp
	./gpiow/Physics/Shapes/ShapeBox.h
	./gpiow/Physics/Shapes/ShapeConvex.cpp
	./gpiow/Physics/Shapes/ShapeConvex.h
	./gpiow/Physics/Shapes/ShapeSphere.cpp
	./gpiow/Physics/Shapes/ShapeSphere.h
)

# book00
add_executable(gpiow_book00 main.cpp gpiow_eigen.h ${gpiow_src_book00} ${gpiow_physics_src} ${gpiow_physics_shapes_src})
target_link_libraries(gpiow_book00 Eigen3::Eigen)

target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow)
target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Scene_Book00)
target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Physics)

# book01
add_executable(gpiow_book01 main.cpp gpiow_eigen.h ${gpiow_src_book01} ${gpiow_physics_src} ${gpiow_physics_shapes_src})
target_link_libraries(gpiow_book01 Eigen3::Eigen)

target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow)
target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Scene_Book01)
target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Physics)

# intersection test with openGJK / libccd
add_executable(test_intersection test_intersection.cpp gpiow_eigen.h ${gpiow_physics_src} ${gpiow_physics_shapes_src} ${opengjk_src} ${libccd_src})
target_link_libraries(test_intersection Eigen3::Eigen)

target_include_directories(test_intersection PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(test_intersection PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow)
target_include_directories(test_intersection PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Physics)

# external project ref
target_include_directories(test_intersection PUBLIC ${CMAKE_BINARY_DIR}/openGJK/include)
target_include_directories(test_intersection PUBLIC ${CMAKE_BINARY_DIR}/libccd/src)

IF(WIN32)
	target_compile_definitions(gpiow_book00 PRIVATE NOMINMAX _USE_MATH_DEFINES)
	target_compile_definitions(gpiow_book01 PRIVATE NOMINMAX _USE_MATH_DEFINES)
	target_compile_definitions(test_intersection PRIVATE NOMINMAX _USE_MATH_DEFINES)
ENDIF(WIN32)

source_group("" FILES main.cpp gpiow_eigen.h)
source_group("gpiow" FILES ${gpiow_src_book00})
source_group("gpiow" FILES ${gpiow_src_book01})
source_group("gpiow\\Physics" FILES ${gpiow_physics_src})
source_group("gpiow\\Physics\\Shapes" FILES ${gpiow_physics_shapes_src})

source_group("" FILES test_intersection.cpp)
source_group("openGJK" FILES ${opengjk_src})
source_group("libccd" FILES ${libccd_src})
