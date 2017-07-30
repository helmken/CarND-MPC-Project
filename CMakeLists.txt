project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
#add_definitions(-std=c++11 -O3 -DHAVE_STDDEF_H)
add_definitions(-std=c++11 -g -DHAVE_STDDEF_H)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources
	src/json.hpp 
	src/main.cpp
	src/MPC.cpp 
	src/MPC.h 
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	include_directories(/usr/local/opt/openssl/include)
	link_directories(/usr/local/opt/openssl/lib)
	link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc 
	${sources}
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(mpc 
	ipopt 
	z 
	ssl 
	uv 
	uWS)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
target_link_libraries(mpc 
	ipopt 
	z 
	ssl 
	uv 
	uWS)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
