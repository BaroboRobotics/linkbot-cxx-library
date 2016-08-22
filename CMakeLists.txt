cmake_minimum_required(VERSION 3.2.1)
project(liblinkbot VERSION 0.3.0 LANGUAGES CXX C)

set(preRelease "rc.1")
# Examples: dev, rc.0, rc.1, some-feature-branch-name
#
# Pre-release data must be a '.'-delimited list of zero or more identifiers composed of one or more
# [0-9A-Za-z-] characters. When linkbotd sorts versions which differ only in their pre-release
# data, the list of pre-release identifiers is sorted lexicographically. Identifiers composed of
# all digits are interpreted as numbers for comparison purposes. Numbers always compare less than
# alphanumeric identifiers (e.g., "a" < "b" and "2" < "10", but "10" < "2a").
#
# Given two version strings which differ only in the presence of pre-release data, the version
# string WITH pre-release data (i.e., `if(preRelease)` evalutes to true) compares less than the
# version string WITHOUT pre-release data. This is because the lack of pre-release data indicates
# an actual release.

set(PROJECT_VERSION_STRING "${PROJECT_VERSION}")
if(preRelease)
    set(PROJECT_VERSION_STRING "${PROJECT_VERSION_STRING}-${preRelease}")
endif()

if(WIN32 AND NOT CMAKE_COMPILER_IS_GNUCXX)
  add_definitions("-D_ALLOW_KEYWORD_MACROS")
  #set_target_properties(${target} PROPERTIES
  #  COMPILE_FLAGS "-D_ALLOW_KEYWORD_MACROS"
  #)
endif()

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.59.0 REQUIRED COMPONENTS system log_setup log filesystem thread)

find_package(cxx-util)
find_package(linkbot-interfaces)
find_package(ribbon-bridge)

set(sources
    src/linkbot.cpp
    src/linkbot.c.cpp
)

add_library(linkbot ${sources})

target_link_libraries(linkbot
    PUBLIC
        barobo::rpc barobo::cxx-util barobo::robot-interface barobo::daemon-interface
        Boost::filesystem Boost::system Boost::log Boost::thread
)

target_include_directories(linkbot
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_compile_definitions(linkbot
    PRIVATE
        LIBRARY_NAME=linkbot
        LIBRARY_VERSION_STRING=${PROJECT_VERSION_STRING}
)

if(MSVC)
    # 64-bit VS compiles need /bigobj
    # Update: 32-bit now needs it too!
    # specific template instantiation might fix this
    set_source_files_properties(src/linkbot.cpp
        PROPERTIES COMPILE_FLAGS "/bigobj")
endif()

set_target_properties(linkbot
    PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON
        POSITION_INDEPENDENT_CODE ON
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        MACOSX_RPATH ON
)

if(WIN32)
    target_link_libraries(linkbot PUBLIC ws2_32 mswsock)
endif()

option(LINKBOT_BUILD_TESTS "Build linkbot tests" ON)
if(LINKBOT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()


install(TARGETS linkbot EXPORT barobo
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

# Boilerplate follows

export(EXPORT barobo
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake"
    NAMESPACE barobo::
)

install(EXPORT barobo
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE barobo::
    DESTINATION cmake
)

file(WRITE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "include(\"\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}-targets.cmake\")"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-configVersion.cmake"
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-configVersion.cmake"
    DESTINATION cmake
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CPACK_DEBIAN_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_DEBIAN_PACKAGE_SECTION "network")

    set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION_STRING}-0)
    set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
    set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
    set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
    # Debian versions must end with a -debian_revision component to allow the use of hyphens in the
    # upstream_version (i.e., DAEMON_VERSION_STRING) component.

    set(CPACK_PACKAGING_INSTALL_PREFIX "/")
    set(CPACK_PACKAGE_VENDOR "Barobo")
    set(CPACK_PACKAGE_CONTACT david@barobo.com)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.19), linkbotd (>= 0.4)")
    # Depends on coreutils for `stdbuf`, udev for `udevadm`.

    get_filename_component(compiler ${CMAKE_CXX_COMPILER} NAME)
    if(${compiler} MATCHES "arm.*")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
    elseif(${compiler} MATCHES "aarch64.*")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
    else()
        execute_process(COMMAND dpkg --print-architecture
                        OUTPUT_VARIABLE DPKG_ARCH
                        OUTPUT_STRIP_TRAILING_WHITESPACE)
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${DPKG_ARCH}")
    endif()
    message(STATUS "Packaging for architecture ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    set (CPACK_PACKAGE_FILE_NAME
    "${CPACK_DEBIAN_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    include(CPack)
endif()
