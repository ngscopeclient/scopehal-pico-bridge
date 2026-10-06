# scopehal-pico-bridge

Socket servers for Pico Technology instruments allowing remote access via libscopehal.


## How to use

##### Prerequisites
* Install the Pico SDK (https://picotech.com/downloads)
* Compile [scopehal-apps](https://www.ngscopeclient.org/manual/GettingStarted.html)
* Compile scopehal-pico-bridge (see below for instructions)
---
1. Connect your PicoScope to the computer.
2. Start the scopehal-pico-bridge (ps6000d.exe). It should open a console window and read "Successfully opened instrument", listing some info about the device like model and serial number.
3. Now start [ngscopeclient](https://github.com/ngscopeclient/scopehal-apps). Select Add / Oscilloscope / Connect... from the top menu. Enter or select the following values:
   * Nickname: (choose a display name for your scope.)
   * Driver: **pico**
   * Transport: **twinlan**
   * Path: **localhost:5025:5026**
4. Click Add.
5. If this is your first time using ngscopeclient, continue reading [the tutorial here](https://www.ngscopeclient.org/manual/Tutorials.html).


## Supported models

Currently supported are these six APIs from Pico Technology: **ps2000a, ps3000a, ps4000a, ps5000a, ps6000a, psospa**.
Note that these are all "A APIs" and thus do not support some older and discontinued models.
As of February 2026 there should be a total of **98** individual devices supported, listed here for reference:

| ps2000aApi | ps3000aApi | psospaApi | ps4000aApi | ps5000aApi | ps6000aApi |
| :---- | :---- | :---- | :---- | :---- | :---- |
| (2205 MSO) | 3203D | 3415E | 4224A | 5242A | 6403E |
| **2205A MSO** | 3203D MSO | 3415E MSO | 4424A | 5242B | 6404E |
| 2206 | 3204A | 3416E | 4444 | 5242D | 6405E |
| 2206A | 3204B | 3416E MSO | **4824** | 5242D MSO | 6406E |
| 2206B | 3204 MSO | 3417E | 4824A | 5243A | 6424E |
| 2206B MSO | 3204D | 3417E MSO |  | 5243B | 6425E |
| 2207 | 3204D MSO | **3418E** |  | 5243D | 6426E |
| 2207A | 3205A | 3418E MSO |  | 5243D MSO | 6428E-D |
| 2207B | 3205B |  |  | 5244A | 6804E |
| 2207B MSO | 3205 MSO |  |  | 5244B | **6824E** |
| 2208 | 3205D |  |  | 5244D |  |
| 2208A | 3205D MSO |  |  | 5244D MSO |  |
| 2208B | 3206A |  |  | 5442A |  |
| 2208B MSO | 3206B |  |  | 5442B |  |
| 2405A | 3206 MSO |  |  | 5442D |  |
| 2406B | 3206D |  |  | 5442D MSO |  |
| 2407B | 3206D MSO |  |  | 5443A |  |
| 2408B | 3207A |  |  | 5443B |  |
|  | 3207B |  |  | 5443D |  |
|  | 3403D |  |  | 5443D MSO |  |
|  | 3403D MSO |  |  | 5444A |  |
|  | 3404A |  |  | 5444B |  |
|  | 3404B |  |  | 5444D |  |
|  | 3404D |  |  | **5444D MSO** |  |
|  | 3404D MSO |  |  |  |  |
|  | 3405A |  |  |  |  |
|  | 3405B |  |  |  |  |
|  | 3405D |  |  |  |  |
|  | 3405D MSO |  |  |  |  |
|  | 3406A |  |  |  |  |
|  | 3406B |  |  |  |  |
|  | 3406D |  |  |  |  |
|  | 3406D MSO |  |  |  |  |

Models shown in bold were used for, and tested during, the development of the pico-bridge.

The model **2205 MSO** is only supported with the Pico SDK up to version **2.2.152.6542**, which will continue to be available from the [Downloads > Discontinued products](https://www.picotech.com/downloads) page of the Pico Technology website.


## How to compile

The process to build from source is basically [the same as for ngscopeclient](https://www.ngscopeclient.org/manual/GettingStarted.html), shown here for **Windows**:

1.
    Download and install MSYS2. You can download it from [msys2.org](https://www.msys2.org/) or [github.com/msys2/msys2-installer/releases](https://github.com/msys2/msys2-installer/releases).
    The following steps can be done in any MSYS-provided shell (it will start up with a **UCRT64** shell after installation).
2.
    Install git and the toolchain:
	```
    pacman -S git wget mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-toolchain
    ```
3.
    Install general dependencies:
    ```
    pacman -S mingw-w64-ucrt-x86_64-libsigc++ mingw-w64-ucrt-x86_64-yaml-cpp mingw-w64-ucrt-x86_64-glfw mingw-w64-ucrt-x86_64-catch mingw-w64-ucrt-x86_64-hidapi mingw-w64-ucrt-x86_64-libpng
    ```
4.
    Install Vulkan dependencies:
    ```
    pacman -S mingw-w64-ucrt-x86_64-vulkan-headers mingw-w64-ucrt-x86_64-vulkan-loader mingw-w64-ucrt-x86_64-shaderc mingw-w64-ucrt-x86_64-glslang mingw-w64-ucrt-x86_64-spirv-tools
    ```
5.
    Check out the code
    ```
    cd ~ 
    git clone --recursive https://github.com/ngscopeclient/scopehal-pico-bridge
    ```
6.
	Add your MSYS2 bin folder to your Windows $PATH variable, i.e. "C:\msys64\ucrt64\bin" if you installed it to "C:\msys64".
7.
    Build manually:
    ```
    cd scopehal-pico-bridge
    mkdir build
    cd build
    cmake ..
    ninja -j4
    ```
8.
    To run scopehal-pico-bridge:
    The binary can be found in the build directory, such as $HOME\scopehal-pico-bridge\build\src\ps6000d.
    Start ps6000d.exe from the Windows Explorer. You may need to move liblog.dll from \build\lib\log into \build\src\ps6000d first.
