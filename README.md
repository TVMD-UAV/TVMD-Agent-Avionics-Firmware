# TVMD-Agent-Avionics

This repository is the firmware of the [TVMD Agent Avionic Board](https://github.com/sciyen/VTswarm/tree/main/pcb/TVMD-agent/TVMD-agent) based on Arduino framework and ESP32-S3.

## Getting Started

### Burning Agent ID to EEPROM
To assign agent id to avionic boards, uncomment these lines in `include/configs.h` and flash the firmware to the board. The agent id will be stored in the EEPROM of the board. This step only needs to be done once for each board. Notice that ID 0 is reserved for the navigator (servo). 
```c
#define WRITE_AGENT_ID
#define AGENT_ID_TO_WRITE 0
```

### Uploading Firmware
To upload the firmware for an avionic client, i.e., agent id other than 0, comment `-D SERVER` in `platformio.ini`. 
```bash
build_flags = 
    -D CORE_DEBUG_LEVEL=5
    -D SERVER
    -I include/
```

### Attach USB to WSL2
Follow this [instruction](https://hackmd.io/@DennisLiu16/rk72brjg2). To bridge USB devices from Windows to WSL2, we need to install usbip in both Windows and WSL2. 

1. Install usbip in windows from [here](https://github.com/dorssel/usbipd-win/releases).
2. Install kernel in WSL
    ```bash
    sudo apt install linux-tools-generic hwdata
    sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
    ```
    Restart the terminal
3. Attach devices
    Run this command in powershell with administrator privileges
    ```bash
    usbipd wsl list
    usbipd wsl attach --distribution Ubuntu-20.04 --busid 2-1
    # or 
    # Auto attach as pluged
    usbipd wsl attach --distribution Ubuntu-20.04 --busid 2-1 --auto-attach
    ```
4. Check attachment
    ```bash
    ls /dev/ttyUSB*
    ls /dev/ttyACM*
    ```