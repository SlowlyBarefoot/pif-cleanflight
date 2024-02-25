COMMON_SRC = \
            build/build_config.c \
            build/debug.c \
            build/version.c \
            $(TARGET_DIR_SRC) \
            main.c \
            ../controller/common/bitarray.c \
            ../controller/common/crc.c \
            ../controller/common/encoding.c \
            ../controller/common/filter.c \
            ../controller/common/huffman.c \
            ../controller/common/huffman_table.c \
            ../controller/common/maths.c \
            ../controller/common/printf.c \
            ../controller/common/streambuf.c \
            ../controller/common/typeconversion.c \
            ../controller/config/config_eeprom.c \
            ../controller/config/feature.c \
            ../controller/config/parameter_group.c \
            ../controller/config/config_streamer.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_i2c_config.c \
            drivers/bus_i2c_busdev.c \
            drivers/bus_i2c_soft.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/bus_spi_soft.c \
            drivers/buttons.c \
            drivers/display.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/light_led.c \
            drivers/resource.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart.c \
            drivers/serial_uart_pinconfig.c \
            drivers/sound_beeper.c \
            drivers/stack_check.c \
            drivers/system.c \
            drivers/timer.c \
            drivers/transponder_ir.c \
            drivers/transponder_ir_arcitimer.c \
            drivers/transponder_ir_ilap.c \
            drivers/transponder_ir_erlt.c \
            ../controller/fc/config.c \
            ../controller/fc/fc_dispatch.c \
            ../controller/fc/fc_hardfaults.c \
            ../controller/fc/fc_msp.c \
            ../controller/fc/fc_msp_box.c \
            ../controller/fc/fc_tasks.c \
            ../controller/fc/runtime_config.c \
            ../controller/io/beeper.c \
            ../controller/io/serial.c \
            ../controller/io/statusindicator.c \
            ../controller/io/transponder_ir.c \
            ../controller/msp/msp_serial.c \
            ../controller/scheduler/scheduler.c \
            ../controller/sensors/battery.c \
            ../controller/sensors/current.c \
            ../controller/sensors/voltage.c \
            ../controller/pif_linker.c

PIF_SRC = \
            ../../pif/source/core/pif.c \
            ../../pif/source/core/pif_obj_array.c \
            ../../pif/source/core/pif_task.c \
            ../../pif/source/core/pif_timer.c

OSD_SLAVE_SRC = \
            ../controller/io/displayport_max7456.c \
            ../controller/osd_slave/osd_slave_init.c \
            ../controller/io/osd_slave.c

FC_SRC = \
            ../controller/fc/fc_init.c \
            ../controller/fc/controlrate_profile.c \
            drivers/camera_control.c \
            drivers/gyro_sync.c \
            drivers/rx_nrf24l01.c \
            drivers/rx_spi.c \
            drivers/rx_xn297.c \
            drivers/pwm_esc_detect.c \
            drivers/pwm_output.c \
            drivers/rx_pwm.c \
            drivers/serial_softserial.c \
            ../controller/fc/fc_core.c \
            ../controller/fc/fc_rc.c \
            ../controller/fc/rc_adjustments.c \
            ../controller/fc/rc_controls.c \
            ../controller/fc/rc_modes.c \
            ../controller/fc/cli.c \
            ../controller/fc/settings.c \
            ../controller/flight/altitude.c \
            ../controller/flight/failsafe.c \
            ../controller/flight/imu.c \
            ../controller/flight/mixer.c \
            ../controller/flight/pid.c \
            ../controller/flight/servos.c \
            ../controller/io/serial_4way.c \
            ../controller/io/serial_4way_avrootloader.c \
            ../controller/io/serial_4way_stk500v2.c \
            ../controller/rx/ibus.c \
            ../controller/rx/jetiexbus.c \
            ../controller/rx/msp.c \
            ../controller/rx/nrf24_cx10.c \
            ../controller/rx/nrf24_inav.c \
            ../controller/rx/nrf24_h8_3d.c \
            ../controller/rx/nrf24_syma.c \
            ../controller/rx/nrf24_v202.c \
            ../controller/rx/pwm.c \
            ../controller/rx/rx.c \
            ../controller/rx/rx_spi.c \
            ../controller/rx/crsf.c \
            ../controller/rx/sbus.c \
            ../controller/rx/spektrum.c \
            ../controller/rx/sumd.c \
            ../controller/rx/sumh.c \
            ../controller/rx/xbus.c \
            ../controller/sensors/acceleration.c \
            ../controller/sensors/boardalignment.c \
            ../controller/sensors/compass.c \
            ../controller/sensors/gyro.c \
            ../controller/sensors/gyroanalyse.c \
            ../controller/sensors/initialisation.c \
            ../controller/blackbox/blackbox.c \
            ../controller/blackbox/blackbox_encoding.c \
            ../controller/blackbox/blackbox_io.c \
            ../controller/cms/cms.c \
            ../controller/cms/cms_menu_blackbox.c \
            ../controller/cms/cms_menu_builtin.c \
            ../controller/cms/cms_menu_imu.c \
            ../controller/cms/cms_menu_ledstrip.c \
            ../controller/cms/cms_menu_misc.c \
            ../controller/cms/cms_menu_osd.c \
            ../controller/cms/cms_menu_vtx_rtc6705.c \
            ../controller/cms/cms_menu_vtx_smartaudio.c \
            ../controller/cms/cms_menu_vtx_tramp.c \
            ../controller/common/colorconversion.c \
            ../controller/common/gps_conversion.c \
            drivers/display_ug2864hsweg01.c \
            drivers/light_ws2811strip.c \
            drivers/serial_escserial.c \
            drivers/sonar_hcsr04.c \
            drivers/vtx_common.c \
            ../controller/flight/navigation.c \
            ../controller/io/dashboard.c \
            ../controller/io/displayport_max7456.c \
            ../controller/io/displayport_msp.c \
            ../controller/io/displayport_oled.c \
            ../controller/io/displayport_rcdevice.c \
            ../controller/io/rcdevice_cam.c \
            ../controller/io/rcdevice.c \
            ../controller/io/rcdevice_osd.c \
            ../controller/io/gps.c \
            ../controller/io/ledstrip.c \
            ../controller/io/osd.c \
            ../controller/sensors/sonar.c \
            ../controller/sensors/barometer.c \
            ../controller/telemetry/telemetry.c \
            ../controller/telemetry/crsf.c \
            ../controller/telemetry/srxl.c \
            ../controller/telemetry/frsky.c \
            ../controller/telemetry/hott.c \
            ../controller/telemetry/smartport.c \
            ../controller/telemetry/ltm.c \
            ../controller/telemetry/mavlink.c \
            ../controller/telemetry/msp_shared.c \
            ../controller/telemetry/ibus.c \
            ../controller/telemetry/ibus_shared.c \
            ../controller/sensors/esc_sensor.c \
            ../controller/io/vtx_string.c \
            ../controller/io/vtx_settings_config.c \
            ../controller/io/vtx_rtc6705.c \
            ../controller/io/vtx_smartaudio.c \
            ../controller/io/vtx_tramp.c \
            ../controller/io/vtx_control.c

COMMON_DEVICE_SRC = \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC)

ifeq ($(OSD_SLAVE),yes)
TARGET_FLAGS := -DUSE_OSD_SLAVE $(TARGET_FLAGS)
COMMON_SRC := $(COMMON_SRC) $(OSD_SLAVE_SRC) $(COMMON_DEVICE_SRC)
else
COMMON_SRC := $(COMMON_SRC) $(FC_SRC) $(COMMON_DEVICE_SRC)
endif


SPEED_OPTIMISED_SRC := ""
SIZE_OPTIMISED_SRC  := ""

ifneq ($(TARGET),$(filter $(TARGET),$(F1_TARGETS)))
SPEED_OPTIMISED_SRC := $(SPEED_OPTIMISED_SRC) \
            ../controller/common/encoding.c \
            ../controller/common/filter.c \
            ../controller/common/maths.c \
            ../controller/common/typeconversion.c \
            drivers/adc.c \
            drivers/buf_writer.c \
            drivers/bus.c \
            drivers/bus_spi.c \
            drivers/exti.c \
            drivers/io.c \
            drivers/pwm_output.c \
            drivers/rcc.c \
            drivers/serial.c \
            drivers/serial_uart.c \
            drivers/system.c \
            drivers/timer.c \
            ../controller/fc/fc_core.c \
            ../controller/fc/fc_tasks.c \
            ../controller/fc/fc_rc.c \
            ../controller/fc/rc_controls.c \
            ../controller/fc/runtime_config.c \
            ../controller/flight/imu.c \
            ../controller/flight/mixer.c \
            ../controller/flight/pid.c \
            ../controller/io/serial.c \
            ../controller/rx/ibus.c \
            ../controller/rx/rx.c \
            ../controller/rx/rx_spi.c \
            ../controller/rx/crsf.c \
            ../controller/rx/sbus.c \
            ../controller/rx/spektrum.c \
            ../controller/rx/sumd.c \
            ../controller/rx/xbus.c \
            ../controller/scheduler/scheduler.c \
            ../controller/sensors/acceleration.c \
            ../controller/sensors/boardalignment.c \
            ../controller/sensors/gyro.c \
            ../controller/sensors/gyroanalyse.c \
            $(CMSIS_SRC) \
            $(DEVICE_STDPERIPH_SRC) \
            drivers/light_ws2811strip.c \
            ../controller/io/displayport_max7456.c \
            ../controller/io/osd.c \
            ../controller/io/osd_slave.c

NOT_OPTIMISED_SRC := $(NOT_OPTIMISED_SRC) \

SIZE_OPTIMISED_SRC := $(SIZE_OPTIMISED_SRC) \
            drivers/bus_i2c_config.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/serial_escserial.c \
            drivers/serial_pinconfig.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/vtx_rtc6705_soft_spi.c \
            drivers/vtx_rtc6705.c \
            drivers/vtx_common.c \
            ../controller/fc/fc_init.c \
            ../controller/fc/cli.c \
            ../controller/fc/settings.c \
            ../controller/config/config_eeprom.c \
            ../controller/config/feature.c \
            ../controller/config/parameter_group.c \
            ../controller/config/config_streamer.c \
            ../controller/io/serial_4way.c \
            ../controller/io/serial_4way_avrootloader.c \
            ../controller/io/serial_4way_stk500v2.c \
            ../controller/io/dashboard.c \
            ../controller/msp/msp_serial.c \
            ../controller/cms/cms.c \
            ../controller/cms/cms_menu_blackbox.c \
            ../controller/cms/cms_menu_builtin.c \
            ../controller/cms/cms_menu_imu.c \
            ../controller/cms/cms_menu_ledstrip.c \
            ../controller/cms/cms_menu_misc.c \
            ../controller/cms/cms_menu_osd.c \
            ../controller/io/vtx_string.c \
            ../controller/io/vtx_settings_config.c \
            ../controller/io/vtx_rtc6705.c \
            ../controller/io/vtx_smartaudio.c \
            ../controller/io/vtx_tramp.c \
            ../controller/io/vtx_control.c
endif #!F1

# check if target.mk supplied
SRC := $(STARTUP_SRC) $(MCU_COMMON_SRC) $(TARGET_SRC) $(VARIANT_SRC)

ifneq ($(DSP_LIB),)

INCLUDE_DIRS += $(DSP_LIB)/Include

SRC += $(DSP_LIB)/Source/BasicMathFunctions/arm_mult_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_rfft_fast_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_cfft_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_rfft_fast_init_f32.c
SRC += $(DSP_LIB)/Source/TransformFunctions/arm_cfft_radix8_f32.c
SRC += $(DSP_LIB)/Source/CommonTables/arm_common_tables.c

SRC += $(DSP_LIB)/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c
SRC += $(DSP_LIB)/Source/StatisticsFunctions/arm_max_f32.c

SRC += $(wildcard $(DSP_LIB)/Source/*/*.S)
endif

ifneq ($(filter ONBOARDFLASH,$(FEATURES)),)
SRC += \
            drivers/flash_m25p16.c \
            ../controller/io/flashfs.c
endif

SRC += $(COMMON_SRC) $(PIF_SRC)

#excludes
SRC   := $(filter-out ${MCU_EXCLUDES}, $(SRC))

ifneq ($(filter SDCARD,$(FEATURES)),)
SRC += \
            drivers/sdcard.c \
            drivers/sdcard_standard.c \
            ../controller/io/asyncfatfs/asyncfatfs.c \
            ../controller/io/asyncfatfs/fat_standard.c
endif

ifneq ($(filter VCP,$(FEATURES)),)
SRC += $(VCP_SRC)
endif
# end target specific make file checks

# Search path and source files for the ST stdperiph library
VPATH        := $(VPATH):$(STDPERIPH_DIR)/src
