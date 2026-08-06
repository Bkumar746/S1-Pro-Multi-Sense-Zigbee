# Build en flash

ESP-IDF 5.3.2:

```sh
source "$HOME/esp/v5.3.2/esp-idf/export.sh"
idf.py set-target esp32c6
idf.py build
```

Zoek de USB-poort en flash een normale update:

```sh
ls /dev/cu.usbmodem*
idf.py -p /dev/cu.usbmodem21101 app-flash
```

Volledige firmware flashen:

```sh
idf.py -p /dev/cu.usbmodem21101 flash
```

Factorybestand voor web.esphome.io maken:

```sh
mkdir -p firmware
idf.py merge-bin -o ../firmware/S1_Pro_Multi_Sense_Zigbee_v1.0.0_factory.bin
```
