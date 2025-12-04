
set(req hcsr04)

idf_component_register(SRCS "hcsr04-example.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${req})
