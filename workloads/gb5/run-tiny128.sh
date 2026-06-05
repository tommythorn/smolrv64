dtc -I dts -O dtb < tiny128.dts > tiny128.dtb;cargo r -r -- -m 512 -d tiny128.dtb,0x9ff00000 fw_payload.elf-6.15.0 tiny128.cpio,0x9f52c000
