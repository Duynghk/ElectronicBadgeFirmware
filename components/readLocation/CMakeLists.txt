idf_component_register(SRCS "readLocation.c"
                    INCLUDE_DIRS "include"
                    REQUIRES libnmea-esp32 "driver" "freertos" "buzzer")
