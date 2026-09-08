cmake_minimum_required(VERSION 3.20)
project(stm32_bno085_spi_shtp VERSION 0.2.0 LANGUAGES C)

option(BNO085_BUILD_TESTS "Build host-side protocol tests" ON)
option(BNO085_BUILD_DOCS "Enable the Doxygen documentation target" OFF)
option(BNO085_BUILD_CALLBACK_PORT "Build the SDK-neutral callback port" ON)

add_library(bno085_core STATIC
    BNO085_Driver/Src/bno085.c
)
add_library(BNO085::Core ALIAS bno085_core)
set_target_properties(bno085_core PROPERTIES EXPORT_NAME Core)
target_include_directories(bno085_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/BNO085_Driver/Inc>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/BNO085_Driver/Port>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(bno085_core PUBLIC c_std_99)

if(BNO085_BUILD_CALLBACK_PORT)
    add_library(bno085_callback_port STATIC
        BNO085_Driver/Port/Callbacks/bno085_port_callbacks.c)
    add_library(BNO085::CallbackPort ALIAS bno085_callback_port)
    set_target_properties(bno085_callback_port PROPERTIES EXPORT_NAME CallbackPort)
    target_include_directories(bno085_callback_port
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/BNO085_Driver/Port>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/BNO085_Driver/Port/Callbacks>
            $<INSTALL_INTERFACE:include>)
    target_compile_features(bno085_callback_port PUBLIC c_std_99)
endif()

if(MSVC)
    target_compile_options(bno085_core PRIVATE /W4)
    if(BNO085_BUILD_CALLBACK_PORT)
        target_compile_options(bno085_callback_port PRIVATE /W4)
    endif()
else()
    target_compile_options(bno085_core PRIVATE -Wall -Wextra -Wpedantic)
    if(BNO085_BUILD_CALLBACK_PORT)
        target_compile_options(bno085_callback_port PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
endif()

include(GNUInstallDirs)
install(TARGETS bno085_core EXPORT BNO085Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES
    BNO085_Driver/Inc/bno085.h
    BNO085_Driver/Port/bno085_port.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(BNO085_BUILD_CALLBACK_PORT)
    install(TARGETS bno085_callback_port EXPORT BNO085Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES BNO085_Driver/Port/Callbacks/bno085_port_callbacks.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()
install(EXPORT BNO085Targets NAMESPACE BNO085::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/BNO085)

if(BNO085_BUILD_TESTS)
    enable_testing()
    add_executable(bno085_parser_tests tests/test_bno085_parser.c)
    target_include_directories(bno085_parser_tests PRIVATE
        BNO085_Driver/Inc BNO085_Driver/Port)
    if(NOT MSVC)
        target_compile_options(bno085_parser_tests PRIVATE
            -Wall -Wextra -Werror -Wpedantic)
        target_link_libraries(bno085_parser_tests PRIVATE m)
    endif()
    add_test(NAME parser_and_fault_injection COMMAND bno085_parser_tests)
endif()

if(BNO085_BUILD_DOCS)
    find_package(Doxygen REQUIRED)
    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/docs")
    configure_file(Doxyfile.in Doxyfile @ONLY)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating BNO085 API documentation")
endif()
