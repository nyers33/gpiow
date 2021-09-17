# https://gamephysicsweekend.github.io/
cmake_minimum_required (VERSION 3.0)
project( gpiow )

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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

add_executable(gpiow_book00 main.cpp gpiow_eigen.h ${gpiow_src_book00} ${gpiow_physics_src} ${gpiow_physics_shapes_src})
target_link_libraries(gpiow_book00 Eigen3::Eigen)

target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow)
target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Scene_Book00)
target_include_directories(gpiow_book00 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Physics)

add_executable(gpiow_book01 main.cpp gpiow_eigen.h ${gpiow_src_book01} ${gpiow_physics_src} ${gpiow_physics_shapes_src})
target_link_libraries(gpiow_book01 Eigen3::Eigen)

target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow)
target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Scene_Book01)
target_include_directories(gpiow_book01 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Physics)

IF(WIN32)
	target_compile_definitions(gpiow_book00 INTERFACE NOMINMAX _USE_MATH_DEFINES)
	target_compile_definitions(gpiow_book01 INTERFACE NOMINMAX _USE_MATH_DEFINES)
ENDIF(WIN32)

source_group("" FILES main.cpp gpiow_eigen.h)
source_group("gpiow" FILES ${gpiow_src_book00})
source_group("gpiow" FILES ${gpiow_src_book01})
source_group("gpiow\\Physics" FILES ${gpiow_physics_src})
source_group("gpiow\\Physics\\Shapes" FILES ${gpiow_physics_shapes_src})

