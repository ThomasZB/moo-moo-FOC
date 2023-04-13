cd cmake-build-debug
make
cd ..
sudo /opt/openocd/bin/openocd -f jlink.cfg -f stm32f1x.cfg -c init -c halt -c "flash write_image erase /home/cat/Project/moo-moo-FOC/program/cmake-build-debug/miniFOC.hex" -c reset -c shutdown -c "set_property KEEPER true [get_ports mcu_TMS]"
