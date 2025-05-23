# zephyr-external
External configuration and drivers for Zephyr

This hosts zephyr external files for building external SoC implementation.

## Support SoC/Board

* akebi96 board
  * SoC: UniPhier LD20
* nx1-ref board
  * SoC: UniPhier NX1

## How to build

On Host:

Create work directory, set virtual python environment, and prepare necessary
tools before zephyr build.
```
mkdir <workdir>
cd <workdir>

python3 -m venv .venv
. .venv/bin/activate
pip install west
```

Initialize and update west with manifest into work directory.
```
west init -m <zephyr-external-url> <workdir>
west update
```

Install necessary python packages and SDK.
```
west packages pip --install
west sdk install -b .
```

Build hello world app.
```
west build -p always -b akebi96 zephyr/samples/hello_world
```

Copy zephyr binary image to tftpboot directory.
```
cp build/zephyr/zephyr.bin /tftpboot
```

## How to run

On U-Boot:

Load zephyr binary image from host into DRAM via TFTP, and
execute it.

```
=> tftpboot 0x8000000 zephyr.bin && dcache flush
=> go 0x80000000
```

Finally, you can see zephyr's startup log on serial console.
```
*** Booting Zephyr OS build v4.1.0-XXXX-gXXXXXXXXXXXX ***
Secondary CPU core 1 (MPID:0x1) is up
Secondary CPU core 2 (MPID:0x100) is up
Secondary CPU core 3 (MPID:0x101) is up
Hello World! akebi96/ld20

uart:~$
```
