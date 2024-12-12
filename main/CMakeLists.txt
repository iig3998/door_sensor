idf_component_register(SRCS "main.c"
					SRCS "components/sensor/sensor.c"
					SRCS "components/wifi/wifi.c"
					#SRCS "components/web_server/web_server.c"
					INCLUDE_DIRS "components/sensor"
					INCLUDE_DIRS "components/wifi"
					# INCLUDE_DIRS "components/web_server"
					PRIV_REQUIRES esp_http_server json
                    REQUIRES driver soc nvs_flash ulp esp_timer esp_wifi esp_http_client)
#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
set(ulp_app_name ulp_${COMPONENT_NAME})
#
# 2. Specify all assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_s_sources "components/ulp/pulse_cnt.S")
#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "main.c")
#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
