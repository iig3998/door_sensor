idf_component_register(SRCS "main.c"
					SRCS "components/common/header.c"
					SRCS "components/common/common.c"
					SRCS "components/sensor/sensor.c"
					SRCS "components/wifi/wifi.c"
					SRCS "components/web_server/web_server.c"
					SRCS "components/nvs_mgmt/nvs_mgmt.c"
					INCLUDE_DIRS "components/common"
					INCLUDE_DIRS "components/sensor"
					INCLUDE_DIRS "components/wifi"
					INCLUDE_DIRS "components/web_server"
					INCLUDE_DIRS "components/nvs_mgmt"
					PRIV_REQUIRES esp_http_server
                    REQUIRES driver soc nvs_flash esp_timer esp_wifi esp_http_client esp_adc)