#!/system/bin/sh
MODDIR=${0%/*}

# Ensure aw8697_haptic custom_wave is writable by system (uid 1000)
for p in /sys/bus/i2c/drivers/aw8697_haptic/*/custom_wave /sys/bus/i2c/drivers/awinic_haptic/*/custom_wave; do
    if [ -e "$p" ]; then
        chown system:system "$p" 2>/dev/null
        chmod 0666 "$p" 2>/dev/null
    fi
done

# Systemlessly bind-mount updated my_product multimedia feature XML before system_server reads it
SRC="$MODDIR/my_product/etc/permissions/oplus.product.feature_multimedia_unique.xml"
DST="/my_product/etc/permissions/oplus.product.feature_multimedia_unique.xml"
if [ -f "$SRC" ] && [ -f "$DST" ]; then
    chcon --reference="$DST" "$SRC" 2>/dev/null
    mount -o bind "$SRC" "$DST" 2>/dev/null
fi
