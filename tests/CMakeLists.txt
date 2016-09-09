add_executable(lavalamp lavalamp.cpp)
target_link_libraries(lavalamp linkbot)
set_target_properties(lavalamp PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
add_test(NAME lavalamp COMMAND lavalamp)

add_executable(testmovement testmovement.cpp)
target_link_libraries(testmovement linkbot)
set_target_properties(testmovement PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
add_test(NAME testmovement COMMAND testmovement)

add_executable(testclinkbot testclinkbot.cpp)
target_link_libraries(testclinkbot linkbot)
set_target_properties(testclinkbot PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
add_test(NAME testclinkbot COMMAND testclinkbot)

add_executable(testclinkboti testclinkboti.cpp)
target_link_libraries(testclinkboti linkbot)
set_target_properties(testclinkboti PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
add_test(NAME testclinkboti COMMAND testclinkboti)

#add_executable(speeddial speeddial.cpp)
#target_link_libraries(speeddial linkbot)
#add_test(NAME speeddial COMMAND speeddial)

add_executable(test_c test_c.cpp)
target_link_libraries(test_c linkbot)
add_test(NAME test_c COMMAND test_c)

#add_executable(tcp tcp.cpp)
#target_link_libraries(tcp linkbot)
#add_test(NAME tcp COMMAND tcp)

add_executable(safetyangles safetyangles.cpp)
target_link_libraries(safetyangles linkbot)
set_target_properties(safetyangles PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED ON)
add_test(NAME safetyangles COMMAND safetyangles)
