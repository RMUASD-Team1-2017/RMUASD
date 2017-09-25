#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

set (TESTAPPDIR ${CMAKE_CURRENT_SOURCE_DIR})
set (TESTAPP_HEADERDIR "$TESTAPP_DIR}/headers")
#set (TESTAPP_HEADERS ${TESTAPP_HEADERDIR}/<something>.hpp)


include_directories(${TESTAPP_HEADERDIR} "${ROOT}/testapp/headers/")

# And now we add any targets that we want
add_subdirectory(sources)
