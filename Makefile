GCC=mingw32-gcc
GPP=mingw32-g++
CFLAGS= -fshow-column -fshow-column -fno-diagnostics-show-caret  -Wall -O2   
CXXFLAGS= -fshow-column -fshow-column -fno-diagnostics-show-caret  -Wall -O2   -std=c++14
LDFLAGS= -static-libstdc++ -static-libgcc -s  
OBJS=Release\flashing_random.o Release\main.o Release\running_rainbow.o Release\running_rainbowv.o Release\drv_speaker.o Release\i2s_ws2812b_drive.o Release\system_nrf52.o Release\ws2812b_drive.o 

all: Release Release\Test.exe
	@echo [100%] Built target Release\Test.exe

clean:
	del ${OBJS} Release\Test.exe

Release\Test.exe: ${OBJS}
	@echo "[ 88%]" Linking executable Release\Test.exe
	@${GPP} ${OBJS} ${LDFLAGS} -o $@

Release:
	mkdir Release

Release\flashing_random.o: flashing_random.c flashing_random.h project.h
	@echo "[  0%]" Building C object Release\flashing_random.o
	@${GCC} ${CFLAGS} -c flashing_random.c -o $@

Release\main.o: main.c project.h notes.h
	@echo "[ 11%]" Building C object Release\main.o
	@${GCC} ${CFLAGS} -c main.c -o $@

Release\running_rainbow.o: running_rainbow.c running_rainbow.h project.h
	@echo "[ 22%]" Building C object Release\running_rainbow.o
	@${GCC} ${CFLAGS} -c running_rainbow.c -o $@

Release\running_rainbowv.o: running_rainbowv.c running_rainbowv.h project.h
	@echo "[ 33%]" Building C object Release\running_rainbowv.o
	@${GCC} ${CFLAGS} -c running_rainbowv.c -o $@

Release\drv_speaker.o: buzzer_driver\drv_speaker.c buzzer_driver\drv_speaker.h buzzer_driver\sounds.h
	@echo "[ 44%]" Building C object Release\drv_speaker.o
	@${GCC} ${CFLAGS} -c buzzer_driver\drv_speaker.c -o $@

Release\i2s_ws2812b_drive.o: i2s_ws2812b_drive\i2s_ws2812b_drive.c i2s_ws2812b_drive\i2s_ws2812b_drive.h
	@echo "[ 55%]" Building C object Release\i2s_ws2812b_drive.o
	@${GCC} ${CFLAGS} -c i2s_ws2812b_drive\i2s_ws2812b_drive.c -o $@

Release\system_nrf52.o: pca10040\s132\arm5_no_packs\RTE\Device\nRF52832_xxAA\system_nrf52.c
	@echo "[ 66%]" Building C object Release\system_nrf52.o
	@${GCC} ${CFLAGS} -c pca10040\s132\arm5_no_packs\RTE\Device\nRF52832_xxAA\system_nrf52.c -o $@

Release\ws2812b_drive.o: ws2812b_driver\ws2812b_drive.c ws2812b_driver\ws2812b_drive.h
	@echo "[ 77%]" Building C object Release\ws2812b_drive.o
	@${GCC} ${CFLAGS} -c ws2812b_driver\ws2812b_drive.c -o $@

