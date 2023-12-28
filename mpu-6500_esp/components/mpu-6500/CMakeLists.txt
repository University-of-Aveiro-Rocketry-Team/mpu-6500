set(component_srcs "mpu-6500.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS ".")

target_compile_definitions(${COMPONENT_LIB} PRIVATE timegm=mktime)
