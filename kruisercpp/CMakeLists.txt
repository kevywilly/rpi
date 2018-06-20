cmake_minimum_required (VERSION 3.7)
project (Kruiser)

#find_package( Boost REQUIRED )
#find_package( OpenCV REQUIRED )

#set(INCLUDE /usr/local/include/neuronic )
set(CMAKE_BUILD_TYPE Release)
set(SOURCES kruiser.cpp tcpserver.cpp)


#include_directories (${INCLUDE})

add_executable(Kruiser ${SOURCES})

#target_link_libraries(Kruiser pthread pigpio ${OpenCV_LIBS})
target_link_libraries(Kruiser pthread pigpio neurocore libyaml.a)

install(TARGETS Kruiser DESTINATION /usr/bin)
