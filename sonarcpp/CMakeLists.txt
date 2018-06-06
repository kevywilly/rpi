cmake_minimum_required (VERSION 3.7)
project (Sonar)

#find_package( Boost REQUIRED )
#find_package( OpenCV REQUIRED )

#set(INCLUDE json network )
set(SOURCES sonar.cpp)


#include_directories (${INCLUDE})

add_executable(Sonar ${SOURCES})

#target_link_libraries(Kruiser pthread pigpio ${OpenCV_LIBS})
target_link_libraries(Sonar pthread pigpio)

install(TARGETS Sonar DESTINATION /usr/bin)