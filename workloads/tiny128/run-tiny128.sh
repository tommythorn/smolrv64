dtc -I dts -O dtb < tiny128.dts > tiny128.dtb;cargo r -r -- -m 128 -d tiny128.dtb,0x87f00000 fw_payload.elf-6.15.0 tiny128.cpio,0x8752c000
