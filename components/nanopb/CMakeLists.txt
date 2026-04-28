set(NANOPB_DIR "${CMAKE_CURRENT_LIST_DIR}/vendor/nanopb")

idf_component_register(
    SRCS
        "${NANOPB_DIR}/pb_common.c"
        "${NANOPB_DIR}/pb_encode.c"
        "${NANOPB_DIR}/pb_decode.c"
    INCLUDE_DIRS
        "${NANOPB_DIR}"
)

