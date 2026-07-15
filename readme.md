# SoC - Empty

> **WARNING — stale committed binaries:** the `.gbl` files in `output_gbl/` were
> committed once at `008e60f` (pre-mouseStim8.1) and never updated. They were
> built with the **old pinout** (PC03=SHDN, PD02/PD03=ELEC, PB00=MAG) and must
> NOT be flashed onto current mouseStim8.1 hardware. The Studio build/debug
> buttons do not regenerate them; only the `create_bl_files.*` scripts do.

## Power regression post-mortem (Jul 2026) — RESOLVED

Symptom: ~4.2 mA idle at cold boot on every firmware version, vs ~1.5-1.9 mA
on validated field units with identical hardware. Fixed: **~1.58 mA at boot**
with firmware v0.2 (`FW2` on the wire identifies fixed devices).

Two independent regressions were introduced when the Studio project was
reconstructed after config/autogen were trashed (both files were gitignored,
so the losses were invisible):

1. **Missing `device_init_lfrco` component** (the ~2 mA). The BGM220SC22HNA is
   crystal-less for the LF domain; `sl_device_init_lfrco()` enables LFRCO
   high-precision mode. Without it, and with `bluetooth_feature_connection`
   present, the BLE stack deems the sleep clock inaccurate and holds a
   permanent EM1 requirement — EM2 is never entered (see
   `config/sl_bluetooth_config.h`, `BT_EM2_LFCLK_REQ_FLAG`). Diagnosed by
   setting `SL_POWER_MANAGER_DEBUG=1` in `config/sl_power_manager_config.h`
   and inspecting `requirement_em_table` (EM1/EM2 counters in
   `sl_power_manager.c`) in a debug session: EM1 count was pinned at 1.
   Fix: `device_init_lfrco` re-added to the `.slcp`.

2. **Wrong linker origin.** The reconstructed `autogen/linkerfile.ld` had
   `FLASH ORIGIN = 0x0` (no bootloader offset), so builds ran standalone at
   0x0, silently clobbering the bootloader. Regeneration restored the correct
   AppLoader layout: `ORIGIN = 0x12000` (bootloader 0x0-0x6000, AppLoader
   0x6000-0x12000, app 0x12000+).

### Flashing requirements (production layout)

1. Mass-erase, then flash the **Bluetooth AppLoader OTA DFU** bootloader.
2. Flash the app (`Creed_DBS_app.hex`/`.s37`, linked at 0x12000).
3. Boot chain: bootloader -> AppLoader -> app. Do not flash app-only onto an
   erased chip and expect OTA to work.

### Related build-system notes

- `SL_DEVICE_INIT_EMU_EM2_DEBUG_ENABLE=0` in `config/sl_device_init_emu_config.h`
  (production power; set to 1 only if a debug session must survive EM2 entry —
  the session dropping at sleep is EM2 working).
- `makefile.defs` contains a guard that only patches in SDK `subdir.mk` files
  when Studio generates a truncated makefile (detected via missing
  `startup_bgm22.o` in `OBJS`). Never include them unconditionally: a healthy
  generation already lists them, and double inclusion links every object twice
  (multiple-definition errors + phantom RAM overflow).
- `autogen/` and `config/` are now git-tracked (see `.gitignore`) so future
  regenerations show up as diffs instead of silent losses.

## Ready-to-connect BLE and LED (v0.4 / `FW4`)

| Mode | Advertising | LED | When |
| --- | --- | --- | --- |
| **ReadyToConnect** | ON | 1 Hz chirp (50 ms) | Boot, disconnect without stim, magnet swipe (incl. during stim) |
| **Connected** | (stack stops adv) | Solid ON (`L` toggles) | BLE connected |
| **Stimulating** | OFF (until magnet) | ~10 s heartbeat only | `G=1` / active output; adv off until magnet or disconnect `G=0` |

1 Hz ready chirp and 10 s stim heartbeat are mutually exclusive. Magnet during
stimulation does **not** stop pulse output — it only re-enables advertising and
switches the LED to 1 Hz (10 s heartbeat halted).

### FUTURE: shelf mode

Shelf mode (not implemented): radio off, LED off, zero advertising. A magnet
swipe is the only wake into ReadyToConnect. The GPIO magnet ISR is the natural
entry point for that behavior.

The Bluetooth SoC-Empty example is a project that you can use as a template for any standalone Bluetooth application.

> Note: this example expects a specific Gecko Bootloader to be present on your device. For details see the Troubleshooting section.

## Getting Started

To learn the Bluetooth technology basics, see [UG103.14: Bluetooth LE Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-14-fundamentals-ble.pdf).

To get started with Silicon Labs Bluetooth and Simplicity Studio, see [QSG169: Bluetooth SDK v3.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg169-bluetooth-sdk-v3x-quick-start-guide.pdf).

The term SoC stands for "System on Chip", meaning that this is a standalone application that runs on the EFR32/BGM and does not require any external MCU or other active components to operate.

As the name implies, the example is an (almost) empty template that has only the bare minimum to make a working Bluetooth application. This skeleton can be extended with the application logic.

The development of a Bluetooth applications consist of three main steps:

* Designing the GATT database
* Responding to the events raised by the Bluetooth stack
* Implementing additional application logic

These steps are covered in the following sections. To learn more about programming an SoC application, see [UG434: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf).

## Designing the GATT Database

The SOC-empty example implements a basic GATT database. GATT definitions (services/characteristics) can be extended using the GATT Configurator, which can be found under Advanced Configurators in the Software Components tab of the Project Configurator. To open the Project Configurator, open the .slcp file of the project.

![Opening GATT Configurator](image/readme_img1.png)

To learn how to use the GATT Configurator, see [UG438: GATT Configurator User’s Guide for Bluetooth SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug438-gatt-configurator-users-guide-sdk-v3x.pdf).

## Responding to Bluetooth Events

A Bluetooth application is event driven. The Bluetooth stack generates events e.g., when a remote device connects or disconnects or when it writes a characteristic in the local GATT database. The application has to handle these events in the `sl_bt_on_event()` function. The prototype of this function is implemented in *app.c*. To handle more events, the switch-case statement of this function is to be extended. For the list of Bluetooth events, see the online [Bluetooth API Reference](https://docs.silabs.com/bluetooth/latest/).

## Implementing Application Logic

Additional application logic has to be implemented in the `app_init()` and `app_process_action()` functions. Find the definitions of these functions in *app.c*. The `app_init()` function is called once when the device is booted, and `app_process_action()` is called repeatedly in a while(1) loop. For example, you can poll peripherals in this function. To save energy and to have this function called at specific intervals only, for example once every second, use the services of the [Sleeptimer](https://docs.silabs.com/gecko-platform/latest/service/api/group-sleeptimer). If you need a more sophisticated application, consider using RTOS (see [AN1260: Integrating v3.x Silicon Labs Bluetooth Applications with Real-Time Operating Systems](https://www.silabs.com/documents/public/application-notes/an1260-integrating-v3x-bluetooth-applications-with-rtos.pdf)).

## Features Already Added to the SOC-Empty Application

The SOC-Empty application is ***almost*** empty. It implements a basic application to demonstrate how to handle events, how to use the GATT database, and how to add software components.

* A simple application is implemented in the event handler function that starts advertising on boot (and on connection_closed event). This makes it possible for remote devices to find the device and connect to it.
* A simple GATT database is defined by adding Generic Access and Device Information services. This makes it possible for remote devices to read out some basic information such as the device name.
* The OTA DFU software component is added, which extends both the event handlers (see *sl_ota_dfu.c*) and the GATT database (see *ota_dfu.xml*). This makes it possible to make Over-The-Air Device-Firmware-Upgrade without any additional application code.

## Testing the SOC-Empty Application

As described above, an empty example does nothing except advertising and letting other devices connect and read its basic GATT database. To test this feature, do the following:

1. Build and flash the SoC-Empty example to your device.
2. Make sure a bootloader is installed. See the Troubleshooting section.
3. Download the **EFR Connect** smartphone app, available on [iOS](https://apps.apple.com/us/app/efr-connect/id1030932759) and [Android](https://play.google.com/store/apps/details?id=com.siliconlabs.bledemo).
4. Open the app and choose the Bluetooth Browser.
   ![EFR Connect start screen](image/readme_img2.png)
5. Now you should find your device advertising as "Empty Example". Tap **Connect**.
   ![Bluetooth Browser](image/readme_img3.png)
6. The connection is opened, and the GATT database is automatically discovered. Find the device name characteristic under Generic Access service and try to read out the device name.
   ![GATT database of the device](image/readme_img4.png)

## Troubleshooting

### Bootloader Issues

Note that Example Projects do not include a bootloader. However, Bluetooth-based Example Projects expect a bootloader to be present on the device in order to support device firmware upgrade (DFU). To get your application to work, you should either 
- flash the proper bootloader or
- remove the DFU functionality from the project.

**If you do not wish to add a bootloader**, then remove the DFU functionality by uninstalling the *Bootloader Application Interface* software component -- and all of its dependants. This will automatically put your application code to the start address of the flash, which means that a bootloader is no longer needed, but also that you will not be able to upgrade your firmware.

**If you want to add a bootloader**, then either 
- Create a bootloader project, build it and flash it to your device. Note that different projects expect different bootloaders:
  - for NCP and RCP projects create a *BGAPI UART DFU* type bootloader
  - for SoC projects on Series 1 devices create a *Bluetooth in-place OTA DFU* type bootloader or any *Internal Storage* type bootloader
  - for SoC projects on Series 2 devices create a *Bluetooth Apploader OTA DFU* type bootloader

- or run a precompiled Demo on your device from the Launcher view before flashing your application. Precompiled demos flash both bootloader and application images to the device. Flashing your own application image after the demo will overwrite the demo application but leave the bootloader in place. 
  - For NCP and RCP projects, flash the *Bluetooth - NCP* demo.
  - For SoC projects, flash the *Bluetooth - SoC Thermometer* demo.

**Important Notes:** 
- when you flash your application image to the device, use the *.hex* or *.s37* output file. Flashing *.bin* files may overwrite (erase) the bootloader.

- On Series 1 devices (EFR32xG1x), both first stage and second stage bootloaders have to be flashed. This can be done at once by flashing the *-combined.s37* file found in the bootloader project after building the project.

- On Series 2 devices SoC example projects require a *Bluetooth Apploader OTA DFU* type bootloader by default. This bootloader needs a lot of flash space and does not fit into the regular bootloader area, hence the application start address must be shifted. This shift is automatically done by the *Apploader Support for Applications* software component, which is installed by default. If you want to use any other bootloader type, you should remove this software component in order to shift the application start address back to the end of the regular bootloader area. Note, that in this case you cannot do OTA DFU with Apploader, but you can still implement application-level OTA DFU by installing the *Application OTA DFU* software component instead of *In-place OTA DFU*.

For more information on bootloaders, see [UG103.6: Bootloader Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-06-fundamentals-bootloading.pdf) and [UG489: Silicon Labs Gecko Bootloader User's Guide for GSDK 4.0 and Higher](https://cn.silabs.com/documents/public/user-guides/ug489-gecko-bootloader-user-guide-gsdk-4.pdf).


### Programming the Radio Board

Before programming the radio board mounted on the mainboard, make sure the power supply switch is in the AEM position (right side) as shown below.

![Radio board power supply switch](image/readme_img0.png)


## Resources

[Bluetooth Documentation](https://docs.silabs.com/bluetooth/latest/)

[UG103.14: Bluetooth LE Fundamentals](https://www.silabs.com/documents/public/user-guides/ug103-14-fundamentals-ble.pdf)

[QSG169: Bluetooth SDK v3.x Quick Start Guide](https://www.silabs.com/documents/public/quick-start-guides/qsg169-bluetooth-sdk-v3x-quick-start-guide.pdf)

[UG434: Silicon Labs Bluetooth ® C Application Developer's Guide for SDK v3.x](https://www.silabs.com/documents/public/user-guides/ug434-bluetooth-c-soc-dev-guide-sdk-v3x.pdf)

[Bluetooth Training](https://www.silabs.com/support/training/bluetooth)

## Report Bugs & Get Support

You are always encouraged and welcome to report any issues you found to us via [Silicon Labs Community](https://www.silabs.com/community).