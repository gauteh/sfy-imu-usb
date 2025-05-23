.DEFAULT_GOAL := build
USB ?= /dev/ttyUSB0
T ?= debug
VARIANT ?= sfy-imu-usb

ifeq "$(T)" "release"
    ELF:= target/thumbv7em-none-eabihf/release/$(VARIANT)
    override CARGO_FLAGS+=--release
else ifeq "$(T)" "r"
    ELF:= target/thumbv7em-none-eabihf/release/$(VARIANT)
    override CARGO_FLAGS+=--release
else
    ELF:= target/thumbv7em-none-eabihf/debug/$(VARIANT)
endif

bump-jlink:
	-python3 ../sfy-code/tools/bump-jlink

build:
	cargo build $(CARGO_FLAGS)

bin: build
	arm-none-eabi-objcopy -S -O binary $(ELF) target/$(VARIANT).bin

flash: bin
	python3 ../sfy-code/tools/svl/svl.py -f target/$(VARIANT).bin $(USB) -v

jlink-flash: bin bump-jlink
	sh jlink-flash.sh target/$(VARIANT).bin

deploy: bin
	python3 ../sfy-code/tools/svl/svl.py -f target/$(VARIANT).bin $(USB) -v

com:
	picocom -e c -b 115200 $(USB)

notecard-com:
	picocom -b 9600 /dev/ttyACM0

defmt-serial-mac:
	(stty speed 115200 >/dev/null && cat) </dev/cu.usbserial-10 | defmt-print -e $(ELF)

defmt-serial:
	socat $(USB),raw,echo=0,b115200 STDOUT | defmt-print -e $(ELF)

gdb-server: bump-jlink
	JLinkGDBServer -select USB -device AMA3B1KK-KBR -endian little -if SWD -noir -noLocalhostOnly -nohalt

gdb-flash:
	gdb-multiarch --command=flash.gdb $(ELF)

gdb-debug:
	gdb-multiarch --command=debug.gdb $(ELF)

defmt-rtt:
	# stdbuf -i0 -e0 -o0 JLinkRTTClient < /dev/null | stdbuf -i0 -e0 -o0 tail -f -n +23 | defmt-print -e target/thumbv7em-none-eabihf/debug/sfy-buoy
	stdbuf -i0 -e0 -o0 nc localhost 19021 | defmt-print -e $(ELF)

rtt:
	# stdbuf -i0 -e0 -o0 JLinkRTTClient < /dev/null | stdbuf -i0 -e0 -o0 tail -f -n +23 | defmt-print -e target/thumbv7em-none-eabihf/debug/sfy-buoy
	stdbuf -i0 -e0 -o0 nc localhost 19021

host-test:
	cargo test
	cargo test --features raw
	cargo test --features fir
	cargo test --features fir,raw
