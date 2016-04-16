# RGB Led and Button Service
Bluetooth Low Energy service for controlling a RGB LED using PWM, and getting notifications on button presses for nRF51 and nRF52 series of Nordic Semiconductor SoCs.

## Prerequisites
* Nordic Semiconductor [nRF5 SDK v11.0.0](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v11.x.x/)
* Nordic Semiconductor [nRF5x Command Line Tools](http://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF52-DK#Downloads)

**Windows**
* Keil uVision IDE [ARM-MDK](https://www.keil.com/demo/eval/arm.htm)
* GNUWin32 [wget](http://gnuwin32.sourceforge.net/packages/wget.htm)
* GNUWin32 [unzip](http://gnuwin32.sourceforge.net/packages/unzip.htm)

**Linux**
* Makefile for gcc will be added later.
* [nRF5x development with gcc and eclipse](https://devzone.nordicsemi.com/tutorials/7/development-with-gcc-and-eclipse/).

## Compilation
* Run bash script `get_sdk.sh` to download and unzip Nordic Semicunductors SDK v11.0.0 for nRF5x series. On windows run it from git-bash.
* Open Keil project in `pca100xx\s13x\arm5_no_packs\` and hit **Build** (F7). Choose `pca10031` if you are using Nordics [nRF51 usb dongle](http://www.nordicsemi.com/eng/Products/nRF51-Dongle) and `pca10041` if you have Nordics [nRF52 DK](http://www.nordicsemi.com/eng/Products/Bluetooth-Smart-Bluetooth-low-energy/nRF52-DK).

## Flashing
* Flash the SoftDevice (bluetooth stack) found in the SDK folder. Replace x depending on if you are using nRF51 s130 or nRF52 s132.
```
cd nRF5_SDK_v11.0.0\components\softdevice\s13x\hex
nrfjprog -f "nrf5x"-e
nrfjprog -f "nrf5x" --program s13x_nrf5x_2.0.0_softdevice.hex
```
* Flash application by hitting **Download** (F8) in Keil. Or by using nrfjprog as bellow.
```
cd nrf5_ble_rgb_btn\pca100xx\s13x\arm5_no_packs\_build
nrfjprog -f "nrf5x" --program ble_rgb_btn_s130.hex
```
