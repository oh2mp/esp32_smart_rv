
# Ready made binaries

Here are the binaries that you can upload to an ESP32 Dev Module with 
[esptool](https://docs.espressif.com/projects/esptool/) without compiling it yourself.

Use this command (everything in one line). Change the port if needed!
The example is for Linux. In Windows the port is probably COM1 or something and in Mac /dev/tty.usbsomething.

```
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 --before default_reset --after hard_reset
  write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB
  0x1000 esp32_smart_rv.ino.bootloader.bin 0x8000 esp32_smart_rv.ino.partitions.bin
  0xe000 esp32_boot_app0.bin 0x10000 esp32_smart_rv.ino.bin 0x310000 esp32_smart_rv.littlefs.bin
```

