python.exe esptool.py --chip esp32 --port %1 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin

@echo off
rem "C:\Users\roman\.platformio\penv\Scripts\python.exe" "C:\Users\roman\.platformio\packages\tool-esptoolpy\esptool.py" --chip esp32 --port "COM4" --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 C:\Users\roman\Documents\GitHub\ev-olution-server\ev-olution-server\.pio\build\upesy_wroom\bootloader.bin 0x8000 C:\Users\roman\Documents\GitHub\ev-olution-server\ev-olution-server\.pio\build\upesy_wroom\partitions.bin 0xe000 C:\Users\roman\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin 0x10000 .pio\build\upesy_wroom\firmware.bin