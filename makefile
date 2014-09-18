
CROSS_COMPILE=mipsel-linux-android-
ARCH=mips

include Makefile

save:
	make savedefconfig
	mv defconfig arch/mips/configs/ci20_android_defconfig

update: save
	make ci20_android_defconfig

copy-kernel: arch/mips/boot/zcompressed/zImage
	cp arch/mips/boot/zcompressed/zImage ../ci20/kernel
	cp arch/mips/boot/zcompressed/zImage $(OUT)/kernel
