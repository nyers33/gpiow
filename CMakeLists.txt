# https://gamephysicsweekend.github.io/
cmake_minimum_required (VERSION 3.0)
project( gpiow )

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(gpiow_src
	./gpiow/Scene.h
	./gpiow/Scene.cpp
)

set(gpiow_physics_src
	./gpiow/Physics/Body.cpp
	./gpiow/Physics/Body.h
	./gpiow/Physics/Broadphase.cpp
	./gpiow/Physics/Broadphase.h
	./gpiow/Physics/Contact.cpp
	./gpiow/Physics/Contact.h
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

add_executable(gpiow main.cpp  gpiow.h ${gpiow_src} ${gpiow_physics_src} ${gpiow_physics_shapes_src})
target_link_libraries(gpiow Eigen3::Eigen)

target_include_directories(gpiow PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(gpiow PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow)
target_include_directories(gpiow PUBLIC ${CMAKE_CURRENT_LIST_DIR}/gpiow/Physics)

IF(WIN32)
	target_compile_definitions(gpiow INTERFACE NOMINMAX _USE_MATH_DEFINES)
ENDIF(WIN32)

source_group("" FILES main.cpp gpiow.h)
source_group("gpiow" FILES ${gpiow_src})
source_group("gpiow\\Physics" FILES ${gpiow_physics_src})
source_group("gpiow\\Physics\\Shapes" FILES ${gpiow_physics_shapes_src})
