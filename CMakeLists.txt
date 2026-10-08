cmake_minimum_required (VERSION 3.13)

project(StepperMotorCoilsAnalyzer)

add_definitions( -DLOGIC2 )

set(CMAKE_OSX_DEPLOYMENT_TARGET "10.14" CACHE STRING "Minimum supported MacOS version" FORCE)

# enable generation of compile_commands.json, helpful for IDEs to locate include files.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# custom CMake Modules are located in the cmake directory.
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(ExternalAnalyzerSDK)

# ExternalAnalyzerSDK selects C++11; the decoder is written against C++17.
set(CMAKE_CXX_STANDARD 17)

# The decoder itself has no dependency on the Analyzer SDK, so it can be unit tested on its own.
add_library(stepper_core STATIC
src/core/EdgeQueue.cpp
src/core/EdgeQueue.h
src/core/MoveGrouper.cpp
src/core/MoveGrouper.h
src/core/PwmEstimator.cpp
src/core/PwmEstimator.h
src/core/StepperDecoder.cpp
src/core/StepperDecoder.h
src/core/StepperPipeline.cpp
src/core/StepperPipeline.h
src/core/StepperWaveform.cpp
src/core/StepperWaveform.h
)
target_include_directories(stepper_core PUBLIC src/core)
set_target_properties(stepper_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(SOURCES
src/IdleFlusher.h
src/StepperMotorCoilsAnalyzer.cpp
src/StepperMotorCoilsAnalyzer.h
src/StepperMotorCoilsAnalyzerResults.cpp
src/StepperMotorCoilsAnalyzerResults.h
src/StepperMotorCoilsAnalyzerSettings.cpp
src/StepperMotorCoilsAnalyzerSettings.h
src/StepperMotorCoilsSimulationDataGenerator.cpp
src/StepperMotorCoilsSimulationDataGenerator.h
)

add_analyzer_plugin(${PROJECT_NAME} SOURCES ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE stepper_core)

option(STEPPER_BUILD_TESTS "Build the decoder unit tests and tools" ON)

if(STEPPER_BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(
        doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG        v2.4.11
        GIT_SHALLOW    True
    )
    FetchContent_GetProperties(doctest)
    if(NOT doctest_POPULATED)
        FetchContent_Populate(doctest)
    endif()

    add_executable(stepper_core_tests tests/core_tests.cpp)
    target_include_directories(stepper_core_tests PRIVATE ${doctest_SOURCE_DIR})
    target_link_libraries(stepper_core_tests PRIVATE stepper_core)
    add_test(NAME stepper_core_tests COMMAND stepper_core_tests)
    set_tests_properties(stepper_core_tests PROPERTIES TIMEOUT 120)

    # Runs the simulation data generator against the real Analyzer SDK library. The Linux SDK
    # library is built against libc++, so linking an executable to it with GCC's libstdc++ fails;
    # the analyzer module is only resolved against it when Logic loads it.
    if(WIN32 OR APPLE)
        add_executable(stepper_simulation_tests
            tests/simulation_tests.cpp
            src/StepperMotorCoilsAnalyzerSettings.cpp
            src/StepperMotorCoilsSimulationDataGenerator.cpp
        )
        target_include_directories(stepper_simulation_tests PRIVATE src ${doctest_SOURCE_DIR})
        target_link_libraries(stepper_simulation_tests PRIVATE stepper_core Saleae::AnalyzerSDK)
        if(WIN32)
            # Multi-config generators put the executable in a per-config folder, away from the copy
            # of the SDK DLL; without it Windows shows a "DLL not found" box and the test hangs.
            add_custom_command(TARGET stepper_simulation_tests POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:Saleae::AnalyzerSDK> $<TARGET_FILE_DIR:stepper_simulation_tests>)
        endif()
        add_test(NAME stepper_simulation_tests COMMAND stepper_simulation_tests)
        set_tests_properties(stepper_simulation_tests PROPERTIES TIMEOUT 120)
    endif()

    # Replays a Logic 2 digital CSV export through the decoder.
    add_executable(stepper_replay tools/replay_csv.cpp)
    target_link_libraries(stepper_replay PRIVATE stepper_core)
endif()
