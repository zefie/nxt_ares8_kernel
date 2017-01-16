#!/bin/bash
DESTDIR=../ares8/boot_remixos_ck/initrd/lib/modules/
rm -f "${DESTDIR}/*"

if [ -z "$1" ] && [ -z "${DESTDIR}" ]; then
	echo "Usage: $0 destdir"
	exit 1;
fi;

if [ ! -z "$1" ]; then
	DESTDIR="$1"
fi

read -r -d '' STOCK_MODULES <<-'STOCK_MODS'
8723bs.ko
atmel_mxt_ts.ko
atomisp-css2400b0_v21.ko
atomisp-css2401a0_legacy_v21.ko
atomisp-css2401a0_v21.ko
bcm_bt_lpm.ko
fps_throttle.ko
gc_class.ko
hdmi_audio.ko
imx1x5.ko
libmsrlisthelper.ko
lm3554.ko
lm3642.ko
mac80211.ko
pax.ko
sensor_general_plugin1_0.ko
sep3_15.ko
SOCWATCH1_5.ko
uvcvideo.ko
videobuf2-core.ko
videobuf2-memops.ko
videobuf2-vmalloc.ko
videobuf-core.ko
videobuf-vmalloc.ko
vtsspp.ko
vxd392.ko
modules.alias
modules.alias.bin
modules.builtin.bin
modules.dep
modules.dep.bin
modules.devname
modules.softdep
modules.symbols
modules.symbols.bin
STOCK_MODS


rm -rf _modtmp
mkdir -p _modtmp
scripts/z_buildenv.sh make INSTALL_MOD_PATH="$(pwd)/_modtmp" modules_install 2>&1 > /dev/null

for m in ${STOCK_MODULES}; do
	# Overrides
	if [ "${m}" == "sensor_general_plugin1_0.ko" ]; then
		FILE=$(find ./_modtmp -name "sensor_general.ko")
	else
		FILE=$(find ./_modtmp -name "${m}")
	fi

	FOUND=0
	if [ -f "${FILE}" ]; then
		FOUND=1
	fi

	if [ "${FOUND}" -eq "1" ]; then
		echo " * [FOUND] ${m}"
		cp -f "${FILE}" "${DESTDIR}/${m}"
	else
		echo " * [MISSN] ${m}" > /dev/stderr
	fi
done;
chmod 644 "${DESTDIR}/"*
cd ..
rm -rf _modtmp