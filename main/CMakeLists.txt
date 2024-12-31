idf_component_register(SRCS "main.c"
					SRCS "components/sensor/sensor.c"
					SRCS "components/wifi/wifi.c"
					#SRCS "components/web_server/web_server.c"
					INCLUDE_DIRS "components/sensor"
					INCLUDE_DIRS "components/wifi"
					# INCLUDE_DIRS "components/web_server"
					PRIV_REQUIRES esp_http_server json
                    REQUIRES driver soc nvs_flash esp_timer esp_wifi esp_http_client)