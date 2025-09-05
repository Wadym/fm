Readme.md

To begin with zephyr framework project:

per https://docs.zephyrproject.org/latest/develop/getting_started/index.html

Ubuntu:
```
sudo apt update
sudo apt upgrade
```

```
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget python3-dev python3-venv python3-tk \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```

```
cmake --version
python3 --version
dtc --version
```
```
python3 -m venv ~/workspace/fm/src/zephyr_based_project/.venv
```

```
source ~/workspace/fm/src/zephyr_based_project/.venv/bin/activate
```
```
pip install west
```
Get the Zephyr source code:

```
west init ~/workspace/fm/src/zephyr_based_project
cd ~/workspace/fm/src/zephyr_based_project/
west update
```

```
west zephyr-export
```

You will get:

```
Zephyr (/home/vg/workspace/fm/src/zephyr_based_project/zephyr/share/zephyr-package/cmake)
has been added to the user package registry in:
~/.cmake/packages/Zephyr

ZephyrUnittest (/home/vg/workspace/fm/src/zephyr_based_project/zephyr/share/zephyrunittest-package/cmake)
has been added to the user package registry in:
~/.cmake/packages/ZephyrUnittest
```

Install Python dependencies using west packages.

```
west packages pip --install
```


Install the Zephyr SDK

```
cd ~/workspace/fm/src/zephyr_based_project/zephyr
```

```
west sdk install
```



Build the Blinky Sample

```
cd ~/zephyrproject/zephyr
west build -p always -b <your-board-name> samples/basic/blinky
```

worked:
```
west build -b teensy40 samples/basic/blinky

west build -b teensy40 blinky
```

```
west flash
```

```
west flash
-- west flash: rebuilding
ninja: no work to do.
-- west flash: using runner teensy
FATAL ERROR: required program teensy_loader_cli not found; install it or add its location to PATH

```

On Ubuntu, you may need to install "libusb-dev" to compile.

```
sudo apt-get install libusb-dev
```

```
clone git@github.com:PaulStoffregen/teensy_loader_cli.git
```

```
cd into teensy_loader_cli

```
make
```

you will get in local folder
```
teensy_loader_cli
```

```
nano ~/.bashrc
```

```
export PATH="$PATH:/path/to/teensy_loader_cli"

export PATH="$PATH:~/workspace/fm/teensy_loader_cli"
export PATH="$PATH:/home/vg/workspace/fm/teensy_loader_cli"

```

```
source ~/.bashrc
```

```
git submodule deinit --force path/to/submodule
git submodule deinit --force src/zephyr_based_project/zephyr

```

```
git submodule add --force https://github.com/Wadym/zephyr.git ./src/zephyr_based_project/zephyr
```

