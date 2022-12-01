
################################################################################
# List of object files from the ESP32 ADF components

ifdef CONFIG_A1S_AUDIO_KIT_V2_2_BOARD
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_board/audio_kit_v2_2_es8388/*.c))
endif
ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_board/lyrat_v4_3/*.c))
endif
ifdef CONFIG_ESP_LYRAT_V4_2_BOARD
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_board/lyrat_v4_2/*.c))
endif
ifdef CONFIG_ESP_LYRATD_MSC_V2_1_BOARD
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_board/lyratd_msc_v2_1/*.c))
endif
ifdef CONFIG_ESP_LYRATD_MSC_V2_2_BOARD
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_board/lyratd_msc_v2_2/*.c))
endif
ifdef CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
ESPADF_AUDIO_BOARD_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_board/lyrat_mini_v1_1/*.c))
endif
ESPADF_AUDIO_HAL_O = $(patsubst %.c,%.o,\
	$(wildcard $(ADFCOMP)/audio_hal/*.c) \
	$(wildcard $(ADFCOMP)/audio_hal/driver/*/*.c) \
	$(wildcard $(ADFCOMP)/audio_hal/driver/*/*/*.c) \
	)

ESPADF_AUDIO_PIPELINE_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_pipeline/*.c))

ESPADF_AUDIO_SAL_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/audio_sal/*.c))

ESPADF_AUDIO_STREAM_O = $(patsubst %.c,%.o,\
	$(ADFCOMP)/audio_stream/fatfs_stream.c \
	$(ADFCOMP)/audio_stream/http_stream.c \
	$(ADFCOMP)/audio_stream/i2s_stream.c \
	$(ADFCOMP)/audio_stream/raw_stream.c \
	$(ADFCOMP)/audio_stream/spiffs_stream.c \
	$(ADFCOMP)/audio_stream/i2s_stream.c \
	$(ADFCOMP)/audio_stream/hls_playlist.c \
	)

ESPADF_DISPLAY_SERVICE_O = $(patsubst %.c,%.o,\
	$(wildcard $(ADFCOMP)/display_service/*.c) \
	$(wildcard $(ADFCOMP)/display_service/led_bar/*.c) \
	$(wildcard $(ADFCOMP)/display_service/led_indicator/*.c) \
	)

ESPADF_ESP_DISPATCHER_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/esp_dispatcher/*.c))

ESPADF_ESP_PERIPHERALS_O = $(patsubst %.c,%.o,\
	$(wildcard $(ADFCOMP)/esp_peripherals/*.c) \
	$(wildcard $(ADFCOMP)/esp_peripherals/driver/i2c_bus/*.c) \
	$(wildcard $(ADFCOMP)/esp_peripherals/lib/*/*.c) \
	)

ESPADF_LIBS_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/esp-adf-libs/esp_codec/*.c))

# adf bt service src files
ESPADF_BT_STREAM_O = $(patsubst %.c,%.o,$(wildcard $(ADFCOMP)/bluetooth_service/*.c))

$(eval $(call gen_espidf_lib_rule,audio_board,$(ESPADF_AUDIO_BOARD_O)))
$(eval $(call gen_espidf_lib_rule,audio_hal,$(ESPADF_AUDIO_HAL_O)))
$(eval $(call gen_espidf_lib_rule,audio_pipeline,$(ESPADF_AUDIO_PIPELINE_O)))
$(eval $(call gen_espidf_lib_rule,audio_sal,$(ESPADF_AUDIO_SAL_O)))
$(eval $(call gen_espidf_lib_rule,audio_stream,$(ESPADF_AUDIO_STREAM_O)))
$(eval $(call gen_espidf_lib_rule,display_service,$(ESPADF_DISPLAY_SERVICE_O)))
$(eval $(call gen_espidf_lib_rule,esp_dispatcher,$(ESPADF_ESP_DISPATCHER_O)))
$(eval $(call gen_espidf_lib_rule,esp_peripherals,$(ESPADF_ESP_PERIPHERALS_O)))
$(eval $(call gen_espidf_lib_rule,esp-adf-libs,$(ESPADF_LIBS_O)))
$(eval $(call gen_espidf_lib_rule,bluetooth_service,$(ESPADF_BT_STREAM_O)))

################################################################################
# List of object files from the ESP32 IDF components which are needed by ADF components

ESPIDF_HTTP_CLIENT_O = $(patsubst %.c,%.o,\
	$(wildcard $(ESPCOMP)/esp_http_client/*.c) \
	$(wildcard $(ESPCOMP)/esp_http_client/lib/*.c) \
	)

ESPIDF_SPIFFS_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/spiffs/spiffs/src/*.c))

# ESPIDF_FATFS_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/fatfs/src/*.c))

ESPIDF_ADC_CAL_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/esp_adc_cal/*.c))

ESPIDF_WEAR_LEVELLING_O = $(patsubst %.cpp,%.o,$(wildcard $(ESPCOMP)/wear_levelling/*.cpp))

ESPIDF_TCP_TRANSPORT_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/tcp_transport/*.c))

ESPIDF_ESP_TLS_O = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/esp-tls/*.c))

ESPIDF_ESP_NGHTTP = $(patsubst %.c,%.o,$(wildcard $(ESPCOMP)/nghttp/port/*.c))

# idf bt component src files
ESPIDF_ESP_BT = $(patsubst %.c,%.o,\
	$(wildcard $(ESPCOMP)/bt/*.c) \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/dm/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/gatt/*.c)                    \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/hh/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/sdp/*.c)                     \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/av/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/ar/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/sys/*.c)                     \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/jv/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/hf_client/*.c)               \
	$(wildcard $(ESPCOMP)/bt/bluedroid/bta/*.c)                         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btcore/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btif/*.c)                        \
	$(wildcard $(ESPCOMP)/bt/bluedroid/device/*.c)                      \
	$(wildcard $(ESPCOMP)/bt/bluedroid/gki/*.c)                         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/hci/*.c)                         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/main/*.c)                        \
	$(wildcard $(ESPCOMP)/bt/bluedroid/osi/*.c)                         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/external/sbc/decoder/srce/*.c)   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/external/sbc/encoder/srce/*.c)   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/core/*.c)                    \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/esp/blufi/*.c)       \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/std/gap/*.c)         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/std/gatt/*.c)        \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/std/a2dp/*.c)        \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/std/avrc/*.c)        \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/std/spp/*.c)         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/std/hf_client/*.c)   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/btc/profile/*.c)                 \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/btm/*.c)                   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/btu/*.c)                   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/gap/*.c)                   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/gatt/*.c)                  \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/hcic/*.c)                  \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/include/*.c)               \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/l2cap/*.c)                 \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/sdp/*.c)                   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/smp/*.c)                   \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/avct/*.c)                  \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/avrc/*.c)                  \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/avdt/*.c)                  \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/a2dp/*.c)                  \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/rfcomm/*.c)                \
	$(wildcard $(ESPCOMP)/bt/bluedroid/stack/*.c)                       \
	$(wildcard $(ESPCOMP)/bt/bluedroid/utils/*.c)                       \
	$(wildcard $(ESPCOMP)/bt/bluedroid/api/*.c)                         \
	$(wildcard $(ESPCOMP)/bt/bluedroid/*.c) 							\
	)

$(eval $(call gen_espidf_lib_rule,esp_http_client,$(ESPIDF_HTTP_CLIENT_O)))
$(eval $(call gen_espidf_lib_rule,spiffs,$(ESPIDF_SPIFFS_O)))
$(eval $(call gen_espidf_lib_rule,fatfs,$(ESPIDF_FATFS_O)))
$(eval $(call gen_espidf_lib_rule,esp_adc_cal,$(ESPIDF_ADC_CAL_O)))
$(eval $(call gen_espidf_lib_rule,wear_levelling,$(ESPIDF_WEAR_LEVELLING_O)))
$(eval $(call gen_espidf_lib_rule,tcp_transport,$(ESPIDF_TCP_TRANSPORT_O)))
$(eval $(call gen_espidf_lib_rule,esp_tls,$(ESPIDF_ESP_TLS_O)))
$(eval $(call gen_espidf_lib_rule,nghttp,$(ESPIDF_ESP_NGHTTP)))
$(eval $(call gen_espidf_lib_rule,bt,$(ESPIDF_ESP_BT)))
