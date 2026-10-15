cmake_minimum_required(VERSION 3.15)
project(mcuno32 VERSION 1.0.0 LANGUAGES C)

add_library(mcuno32 STATIC
    src/chorus_mn3009.c
    src/ir3109_vcf.c
    src/juno_engine.c
    src/master_hpf.c
    src/polyblep.c
    src/voice.c
)

target_include_directories(mcuno32
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(mcuno32 PUBLIC c_std_99)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mcuno32 PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(mcuno32 PUBLIC ${MATH_LIBRARY})
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()
    add_executable(mcuno32_test tests/test_smoke.c)
    target_link_libraries(mcuno32_test PRIVATE mcuno32)
    add_test(NAME smoke_test COMMAND mcuno32_test)
endif()
