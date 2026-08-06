set(BSEC2_COMMIT "4f559a6aa450f0ce436e602b42dc384f52128c66")
set(BSEC2_DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/_deps/bsec2")
set(BSEC2_ARCHIVE "${BSEC2_DOWNLOAD_DIR}/${BSEC2_COMMIT}.tar.gz")
set(BSEC2_SOURCE_DIR "${BSEC2_DOWNLOAD_DIR}/Bosch-BSEC2-Library-${BSEC2_COMMIT}")

if(NOT EXISTS "${BSEC2_SOURCE_DIR}/src/esp32c3/libalgobsec.a")
    file(MAKE_DIRECTORY "${BSEC2_DOWNLOAD_DIR}")
    file(
        DOWNLOAD
        "https://github.com/boschsensortec/Bosch-BSEC2-Library/archive/${BSEC2_COMMIT}.tar.gz"
        "${BSEC2_ARCHIVE}"
        EXPECTED_HASH "SHA256=c8c1be93b64e485bdc6c169cf0b69c0278fc06397f99bf776732e3d3147aad58"
        TLS_VERIFY ON
        STATUS BSEC2_DOWNLOAD_STATUS
    )
    list(GET BSEC2_DOWNLOAD_STATUS 0 BSEC2_DOWNLOAD_RESULT)
    list(GET BSEC2_DOWNLOAD_STATUS 1 BSEC2_DOWNLOAD_MESSAGE)
    if(NOT BSEC2_DOWNLOAD_RESULT EQUAL 0)
        message(FATAL_ERROR "BSEC2 download failed: ${BSEC2_DOWNLOAD_MESSAGE}")
    endif()
    file(ARCHIVE_EXTRACT INPUT "${BSEC2_ARCHIVE}" DESTINATION "${BSEC2_DOWNLOAD_DIR}")
endif()

idf_component_register(
    SRCS "${BSEC2_SOURCE_DIR}/src/bsec2.cpp"
    INCLUDE_DIRS "${BSEC2_SOURCE_DIR}/src"
    REQUIRES bme68x
)

add_prebuilt_library(
    bsec2_algorithm
    "${BSEC2_SOURCE_DIR}/src/esp32c3/libalgobsec.a"
)
target_link_libraries(${COMPONENT_LIB} PRIVATE bsec2_algorithm)
