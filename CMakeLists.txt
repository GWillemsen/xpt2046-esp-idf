set(SRCS "src/xpt2046.c")

idf_component_register(
        SRCS ${SRCS}
        INCLUDE_DIRS ./include
)