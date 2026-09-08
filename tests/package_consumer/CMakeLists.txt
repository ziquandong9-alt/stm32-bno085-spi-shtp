cmake_minimum_required(VERSION 3.20)
project(bno085_package_consumer LANGUAGES C)

find_package(BNO085 0.2 CONFIG REQUIRED)
add_executable(package_consumer main.c)
target_link_libraries(package_consumer PRIVATE
    BNO085::Core BNO085::CallbackPort)
