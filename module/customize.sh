#!/system/bin/sh
ui_print "- Installing O-Haptics Bridge for Xiaomi 12X (psyche)"
set_perm_recursive "$MODPATH" 0 0 0755 0644
set_perm "$MODPATH/system/vendor/bin/hw/vendor.xiaomi.hardware.vibratorfeature.service" 0 2000 0755 u:object_r:hal_vibrator_default_exec:s0
set_perm "$MODPATH/post-fs-data.sh" 0 0 0755
set_perm "$MODPATH/service.sh" 0 0 0755
ui_print "- Waveforms installed: $(ls "$MODPATH/system/vendor/firmware/ohaptics/"*_RTP.bin 2>/dev/null | wc -l)"
