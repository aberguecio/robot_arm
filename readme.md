subir:
ampy --port COM9 put main.py

formatear:
esptool.exe --chip esp32 --port COM9 erase_flash
esptool.exe --chip esp32 --port COM9 write_flash -z 0x1000 ../ESP32_v1.24.0.bin
esptool.exe --chip esp32 --port COM11 write_flash -z 0x1000 ../micropython_camera_feeeb5ea3_esp32_idf4_4.bin

espcam COM11

