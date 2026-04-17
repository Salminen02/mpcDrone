cmake_minimum_required(VERSION 3.10)
project(MissileSimulation)

# Build asetukset
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -march=native)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 1. Etsi paketit
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Acados polut
set(ACADOS_DIR "${CMAKE_SOURCE_DIR}/src/drone/acados")
set(ACADOS_GENERATED_DIR "${CMAKE_SOURCE_DIR}/src/drone/c_generated_code")

# 2. Otsikkotiedostojen polut
include_directories(
    include
    /usr/local/include
    ${CMAKE_SOURCE_DIR}/src
    /usr/include/eigen3
    ${ACADOS_DIR}/include
    ${ACADOS_DIR}/include/acados
    ${ACADOS_DIR}/include/blasfeo/include
    ${ACADOS_DIR}/include/hpipm/include
    ${ACADOS_GENERATED_DIR}
)

# Generoi acados C-koodi Acados.py:llä ennen buildia
add_custom_command(
    OUTPUT ${ACADOS_GENERATED_DIR}/acados_solver_quadrotor_mpcc.c
    COMMAND python3 ${CMAKE_SOURCE_DIR}/src/drone/Acados.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/src/drone
    DEPENDS ${CMAKE_SOURCE_DIR}/src/drone/Acados.py
    COMMENT "Generoidaan acados C-koodi Acados.py:llä..."
)

# 3. Lähdekoodit
set(SOURCES
    src/main.cpp
    src/drone/controller.cpp
    src/drone/controller.hpp
    src/drone/hardDrone.cpp
    src/drone/hardDrone.hpp
    src/simulation/sim.cpp
    src/simulation/simThread.cpp
    src/raylibGraphics/raylibGraphics.cpp
    # Acados-generoitu C-koodi
    ${ACADOS_GENERATED_DIR}/acados_solver_quadrotor_mpcc.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_0_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_0_fun_jac.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_0_fun_jac_hess.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_fun_jac.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_fun_jac_hess.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_e_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_e_fun_jac.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_e_fun_jac_hess.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_model/quadrotor_mpcc_expl_ode_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_model/quadrotor_mpcc_expl_vde_forw.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_model/quadrotor_mpcc_expl_vde_adj.c
    # Acados constraints
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_fun_jac_uxt_zt.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_e_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_e_fun_jac_uxt_zt.c
)

# Merkitään generoidut tiedostot GENERATED jotta cmake ei vaadi niitä konfigurointivaiheessa
set_source_files_properties(
    ${ACADOS_GENERATED_DIR}/acados_solver_quadrotor_mpcc.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_0_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_0_fun_jac.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_0_fun_jac_hess.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_fun_jac.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_fun_jac_hess.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_e_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_e_fun_jac.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_cost/quadrotor_mpcc_cost_ext_cost_e_fun_jac_hess.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_model/quadrotor_mpcc_expl_ode_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_model/quadrotor_mpcc_expl_vde_forw.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_model/quadrotor_mpcc_expl_vde_adj.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_fun_jac_uxt_zt.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_e_fun.c
    ${ACADOS_GENERATED_DIR}/quadrotor_mpcc_constraints/quadrotor_mpcc_constr_h_e_fun_jac_uxt_zt.c
    PROPERTIES GENERATED TRUE
)

# 4. Luo suoritustiedosto
add_executable(drone_sim ${SOURCES})

# 5. Linkitä kirjastot
target_link_libraries(drone_sim 
    Eigen3::Eigen
    /usr/local/lib/libraylib.a
    # Acados
    ${ACADOS_DIR}/lib/libacados.so
    ${ACADOS_DIR}/lib/libhpipm.so
    ${ACADOS_DIR}/lib/libblasfeo.so
    ${ACADOS_DIR}/lib/libqpOASES_e.so
    GL
    m
    pthread
    dl
    rt
    X11
)

# Acados lib-kansio runtime-hakupolkuun
set_target_properties(drone_sim PROPERTIES
    BUILD_RPATH "${ACADOS_DIR}/lib"
    INSTALL_RPATH "${ACADOS_DIR}/lib"
)