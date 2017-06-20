cmake_minimum_required(VERSION 2.8.3)
project(arc_mpc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS roscpp arc_tools geometry_msgs visualization_msgs nav_msgs ackermann_msgs catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
	src/arc_mpc.cpp
	arc_solver/arc_solver_model_20.c
	arc_solver/arc_solver_model_1.c
	arc_solver/arc_solver/obj_target/arc_solver.o
##	arc_solver/arc_solver/lib_target/libarc_solver.so
	alglib/src/alglibinternal.cpp
	alglib/src/alglibmisc.cpp
	alglib/src/ap.cpp
	alglib/src/dataanalysis.cpp
	alglib/src/diffequations.cpp
	alglib/src/fasttransforms.cpp
	alglib/src/integration.cpp
	alglib/src/interpolation.cpp
	alglib/src/linalg.cpp
	alglib/src/optimization.cpp
	alglib/src/solvers.cpp
	alglib/src/specialfunctions.cpp
	alglib/src/statistics.cpp
)
#add_library(${PROJECT_NAME} OBJECT arc_solver/arc_solver/obj/arc_solver.o)
#add_library( src/arc_mpc.cpp)
#add_library(arc_solver_model_9.o src/arc_mpc.cpp)
#include_directories(include ${catkin_INCLUDE_DIRS})





## Declare a C++ executable
add_executable(arc_mpc_controller src/arc_mpc_node.cpp
#	arc_mpc_controller arc_solver/arc_solver_model_9.c
#	arc_mpc_controller arc_solver/arc_solver_model_1.c
#	arc_mpc_controller arc_solver/arc_solver_casadi2forces.c
)

## Add cmake target dependencies of the executable
## same as for the library above
##add_dependencies(pure_pursuit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arc_mpc_controller
   ${catkin_LIBRARIES}
${PROJECT_NAME}
#	arc_solver/arc_solver/obj/arc_solver.o
#	arc_solver/arc_solver/lib/arc_solver.so
 )

cs_install()
cs_export()
