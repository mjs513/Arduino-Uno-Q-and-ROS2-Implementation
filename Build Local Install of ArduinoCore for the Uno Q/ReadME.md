# How to Install/Build ArduinoCore-Zephry on Arduino Uno Q 4GB

**Purpose**

The document explains how to manually install, build, and deploy the ArduinoCore‑Zephyr environment for the Arduino Uno Q (4GB model only), including workarounds for current repository issues.

Two methods are provided to build and install the ArduinoCore for the Arduino Uno Q.  
- Method 1. Install per the Core build instructions which will take more space on the Q and
- Method 2. Using symbolic links.

# **WARNING --- DO NOT ATTEMPT TO DO THIS ON A ARDUINO UNO Q 2gb**

# Installation based on [ArduinoCore-zephyr ReadME](https://github.com/arduino/ArduinoCore-zephyr)

## 1. Repository Setup
```bash
mkdir my_new_zephyr_folder && cd my_new_zephyr_folder
git clone https://github.com/arduino/ArduinoCore-zephyr
```

or if you want to install submodules: RPC_Lite and RouterBridge
```
git clone --recurse-submodules https://github.com/arduino/ArduinoCore-zephyr
```

** NOTE:  using `--recursive-submodules does not work right now since there are URL issues in `.gitmodules` and `install.sh` for the Arduino api.  See below for installation after installing the core files.**

## 2. System Dependencies
Install Dependencies:

```bash
sudo apt install python3-pip python3-setuptools python3-venv build-essential git cmake ninja-build zstd jq rsync
```

## 3. Create a arduino uno q bootstrap script
```
cd extra
touch bootstrap_q_only.sh
nano bootstrap_q_only.sh
```
Past the following into `bootstrap_q_only.sh`
```
#!/bin/bash

set -e

if [ ! -f platform.txt ]; then
  echo Launch this script from the root core folder as ./extra/bootstrap.sh
  exit 2
fi

#NEEDED_HALS=$(grep 'build.zephyr_hals=' boards.txt | cut -d '=' -f 2 | xargs -n 1 echo | sort -u)

# Modified to only download and STM32 HAL
HAL_FILTER="-hal_.*,+hal_stm32"
#HAL_FILTER="-hal_.*"
#for hal in $NEEDED_HALS; do
#  HAL_FILTER="$HAL_FILTER,+$hal"
#done

python3 -m venv venv
source venv/bin/activate
pip install west protobuf grpcio-tools
west init -l .
west config manifest.project-filter -- "$HAL_FILTER"
west update "$@"
west zephyr-export
pip install -r ../zephyr/scripts/requirements-base.txt
west sdk install --version 0.16.8 -t arm-zephyr-eabi
west blobs fetch $NEEDED_HALS
```
then change mode to exectuable
```
chmod +x bootstrap_q_only.sh
```

## 3. Run the ```bootstrap``` script
```bash
cd ArduinoCore-zephyr (if necessary)
./extra/bootstrap_q_only.sh
```

NOTE:  If you get an error message at this point:
1.  change directory to /my_new_zephyr_folder and do a `rm -r .west
2.  change back to the  ArduinoCore-zephyr directory and execute `./extra/bootstrap_q_only.sh` again.

## 4. Fixing Submodule URLs 

Note until .gitmodules is updated use first method and moddify the downloaded .gitmodules files
```
[submodule "libraries/Arduino_RouterBridge"]
        path = libraries/Arduino_RouterBridge
        url = https://github.com/arduino-libraries/Arduino_RouterBridge.git
[submodule "libraries/Arduino_RPClite"]
        path = libraries/Arduino_RPClite
        url = https://github.com/arduino-libraries/Arduino_RPClite.git
```
and run
```bash
git submodule update --init
```

To update the Core Api you have to run `install.sh` however this file needs to modified as well for now to correct the URL to
```bash
git clone https://github.com/arduino/ArduinoCore-API.git;
```

## 5. Rebuilding the Core

The loader is compiled for each board by running the `./extra/build.sh` script.
The target can be specified either with the Arduino board name (as defined in
boards.txt), or with the Zephyr board name and any additional arguments that
may be required by the Zephyr build system.

To build for Arduino Uno Q:

```bash
./extra/build.sh arduino_uno_q
```

## 6. Installing the Development Platform

See this post in the Arduino AppLab forum:
https://forum.arduino.cc/t/install-sources-built-zephyr-on-uno-q-for-applab/1429150/7?_gl=1*17o7n6q*_up*MQ..*_ga*NTc1NDk0Njk5LjE3NzA5MDQ1NzE.*_ga_NEXN8H46L5*czE3NzA5MDQ1NzAkbzEkZzAkdDE3NzA5MDQ1NzAkajYwJGwwJGgxNzc4MzYxMjI.

**Recommended Approach**

- Manually install the development version of the platform under the sketchbook as usual, but name the vendor folder arduino
- Disable the release installation of the platform

**Manual Installation Structure**
The structure of the manual installation of the platform should be like so:
```
~/Arduino/
├── hardware/
│   ├── arduino/
│   │   └── zephyr/
│   │       ├── boards.txt
│           ...
```

**Disable Release Installation**
```
mv ~/.arduino15/packages/arduino/hardware/zephyr ~/.arduino15/packages/arduino/hardware/zephyr.disable
```
**Enable Local Installation**
```
mv ~/Arduino/hardware/arduino/zephyr.disable ~/Arduino/hardware/arduino/zephyr
```

## Copying Files Into the Local Install

This should work whether you have previously created the local install directory or not.

```
rsync -av  --include='*.txt' --exclude='*' --update ~/my_new_zephyr_folder/ArduinoCore-zephyr/ ~/Arduino/hardware/arduino/zephyr

rsync -av --update ~/my_new_zephyr_folder/ArduinoCore-zephyr/libraries/ ~/Arduino/hardware/arduino/zephyr/libraries/

rsync -av --update --exclude='CMakeLists.txt' --exclude='arduino/api' ~/my_new_zephyr_folder/ArduinoCore-zephyr/cores/ ~/Arduino/hardware/arduino/zephyr/cores/

rsync -av --update --include="_ldscripts/***" --exclude='*' ~/my_new_zephyr_folder/ArduinoCore-zephyr/variants/ ~/Arduino/hardware/arduino/zephyr/variants/

rsync -av --update --include='arduino_uno_q_stm32u585xx/***' --exclude='*' ~/my_new_zephyr_folder/ArduinoCore-zephyr/variants/  ~/Arduino/hardware/arduino/zephyr/variants/

rsync -av --update --include="*/" --include="*arduino_uno_q*" --exclude='*' ~/my_new_zephyr_folder/ArduinoCore-zephyr/firmwares/ ~/Arduino/hardware/arduino/zephyr/firmwares
```

## 8. Flashing the Loader

Use Arduino CLI with a J-Link programmer:
```
arduino-cli burn-bootloader -b arduino:zephyr:unoq -P jlink
```
## 9. Restore Released ArduinoCore-zephyr
**Restore Release Version**
```
mv ~/.arduino15/packages/arduino/hardware/zephyr.disable ~/.arduino15/packages/arduino/hardware/zephyr

mv ~/Arduino/hardware/arduino/zephyr ~/Arduino/hardware/arduino/zephyr.disable
```

**Burn Bootloader**
```
arduino-cli burn-bootloader -b arduino:zephyr:unoq -P jlink
```

# Alternate Installation Using GIT
REF: https://forum.arduino.cc/t/install-sources-built-zephyr-on-uno-q-for-applab/1429150/24?_gl=1*17o7n6q*_up*MQ..*_ga*NTc1NDk0Njk5LjE3NzA5MDQ1NzE.*_ga_NEXN8H46L5*czE3NzA5MDQ1NzAkbzEkZzAkdDE3NzA5MDQ1NzAkajYwJGwwJGgxNzc4MzYxMjI.

This method creates a symbolic link between where you built the core and where the required local install directory.

## 1. Clone the ArduinoCore-zephyr repo to a git directory
```
mkdir git && cd git
git clone --branch main https://github.com/arduino/ArduinoCore-zephyr
```

## 2. Install dependeciences
```
sudo apt install python3-pip python3-setuptools python3-venv build-essential git cmake ninja-build zstd jq rsync
```
## 3. Create a arduino uno q bootstrap script
```
cd extra
touch bootstrap_q_only.sh
nano bootstrap_q_only.sh
```
Past the following into `bootstrap_q_only.sh`
```
#!/bin/bash

set -e

if [ ! -f platform.txt ]; then
  echo Launch this script from the root core folder as ./extra/bootstrap.sh
  exit 2
fi

#NEEDED_HALS=$(grep 'build.zephyr_hals=' boards.txt | cut -d '=' -f 2 | xargs -n 1 echo | sort -u)

# Modified to only download and STM32 HAL
HAL_FILTER="-hal_.*,+hal_stm32"
#HAL_FILTER="-hal_.*"
#for hal in $NEEDED_HALS; do
#  HAL_FILTER="$HAL_FILTER,+$hal"
#done

python3 -m venv venv
source venv/bin/activate
pip install west protobuf grpcio-tools
west init -l .
west config manifest.project-filter -- "$HAL_FILTER"
west update "$@"
west zephyr-export
pip install -r ../zephyr/scripts/requirements-base.txt
west sdk install --version 0.16.8 -t arm-zephyr-eabi
west blobs fetch $NEEDED_HALS
```
then change mode to exectuable
```
chmod +x bootstrap_q_only.sh
```

## 4. Run the `bootstrap` script
```bash
cd ArduinoCore-zephyr (if necessary)
./extra/bootstrap_q_only.sh
```

NOTE:  If you get an error message at this point:
1.  change directory to /my_new_zephyr_folder and do a `rm -r .west
2.  change back to the  ArduinoCore-zephyr directory and execute `./extra/bootstrap_q_only.sh` again.

## 5. Setup Local Installation Configuration
In this method a symbolic link is used

1. Create the directory:
```
mkdir -p  ~/Arduino/hardware/arduino
```
2. `cd` to the directory
```
cd  ~/Arduino/hardware/arduino 
```
4. Create a symbolic link to your github project.

```
ln -s /home/arduino/git/ArduinoCore-zephyr/ zephyr
```

Note: if you do not change to the `~/Arduino/hardware/arduino` the link command would be:
```
ln -s /home/arduino/git/ArduinoCore-zephyr/  ~/Arduino/hardware/arduino/zephyr
```
5. Disable the released version:
```
mv ~/.arduino15/packages/arduino/hardware/zephyr.disable ~/.arduino15/packages/arduino/hardware/zephyr
```

## 6. Fixing libraries:
```
cd ~/git
git clone https://github.com/arduino-libraries/Arduino_RouterBridge.git
git clone https://github.com/arduino-libraries/Arduino_RPClite.git
cd arduino@Uno-Q4:~/git$ cd ArduinoCore-zephyr/libraries/
rmdir Arduino_RouterBridge
ln -s ~/git/Arduino_RouterBridge
rmdir Arduino_RPClite
ln -s ~/git/Arduino_RPClite
```

## 7. Flashing the Loader

Use Arduino CLI with a J-Link programmer:
```
arduino-cli burn-bootloader -b arduino:zephyr:unoq -P jlink
```
## 8. Restore Released ArduinoCore-zephyr
**Restore Release Version**
```
mv ~/.arduino15/packages/arduino/hardware/zephyr.disable ~/.arduino15/packages/arduino/hardware/zephyr

mv ~/Arduino/hardware/arduino/zephyr ~/Arduino/hardware/arduino/zephyr.disable
```

**Burn Bootloader**
```
arduino-cli burn-bootloader -b arduino:zephyr:unoq -P jlink
```

# Troubleshooting
## 1. When first do the ./bootstrap.sh command it failed to download one of the modules requested had to

1. delete .west directory in the /my_new_zephyr_folder directory and
2. delete venv directory in the my_new_zephyr_folder/ArduinoCore-zephyr and then re-ran the ./bootstrap.sh script.

You you get any errors at this step this may fix it.
## 2. When Running the new installation if you receive build errors there are 2 things to try and then rebuild:  
1. delete the `.cache` file for the app and rebuild
2. Run the following command:
```
arduino-app-cli system cleanup
```

# Credits
See [Install sources built zephyr on UNO Q for applab](https://forum.arduino.cc/t/install-sources-built-zephyr-on-uno-q-for-applab/1429150/43?_gl=1*17o7n6q*_up*MQ..*_ga*NTc1NDk0Njk5LjE3NzA5MDQ1NzE.*_ga_NEXN8H46L5*czE3NzA5MDQ1NzAkbzEkZzAkdDE3NzA5MDQ1NzAkajYwJGwwJGgxNzc4MzYxMjI.)

