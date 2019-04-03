# serial_ti_radar: UAV target tracking and pursuit using mmWave radar
# The UAV running PX4 firmware enters into the offboard mode, track and pursuit a person/uav
# serial_radar
# This repo was tested in Ubuntu 16.04 and ros kinetic
# This repo provides a serial communication driver for TI IWR1642 mmWave sensor
# The TI IWR1642 was installed with the people counting demo firmware
# serial_ti_radar_config.cpp send the cfg command to the radar
# serial_ti_radar_read.cpp extracts the radar info
# frame_transformation.cpp UAV track and pursuit target in autonomous offboard mode
