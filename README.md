# O-Haptics Bridge for Xiaomi 12X (`psyche`) — ColorOS 17

Adapted from [nischay12089/Oplus-vibrator-feature](https://github.com/nischay12089/Oplus-vibrator-feature) for Xiaomi 12X (`psyche`, SM8250) running ColorOS 17 on the `4.19` `aw8697_haptic` kernel.

## What Was Adapted for `psyche`
1. **Dual AIDL service registration + VINTF overlay:** `psyche`'s stock vendor HAL only registered `IVibrator/vibratorfeature`, leaving ColorOS 17's `VibratorManagerService` with `vibratorIds = []`. This module registers both `android.hardware.vibrator.IVibrator/default` and `IVibrator/vibratorfeature` (each with the `IOplusVibrator` V2 RichTap HE binder extension) and overlays `/vendor/etc/vintf/manifest/vendor.xiaomi.hardware.vibratorfeature.service.xml`.
2. **`aw8697_haptic` motor backend:** Targets `/dev/input/event*` (`aw8697_haptic`) + `/sys/bus/i2c/drivers/aw8697_haptic/2-005a/custom_wave`, reads calibrated `f0` from `f0_value` (`1701` -> `170.1 Hz`) without buzzing at boot, and supports runtime override of `custom_wave` ID via `persist.vendor.ohaptics.custom_wave_id` (default `197`).
3. **Isolated waveform directory:** Extracts upstream's ColorOS `*_RTP.bin` waveforms (tuned for 168.5 Hz, within 1% of `psyche`'s 170.1 Hz `f0`) into `/vendor/firmware/ohaptics/` so stock `/vendor/firmware/` files stay untouched.
4. **Systemless `my_product` overlay:** `post-fs-data.sh` bind-mounts the updated `oplus.product.feature_multimedia_unique.xml` (dropping `oplus.software.audio.haptic_channel_support` and adding `oplus.software.vibrator.long_press_home_wave_support`).

## Build via GitHub Actions
1. Push the contents of this archive to a GitHub repository (public or private).
2. Open the **Actions** tab and run **Build O-Haptics Psyche ColorOS 17 Module**.
3. Download the `OHaptics-Psyche-ColorOS17-v1.0` artifact, extract the outer GitHub artifact zip, and flash `OHaptics-Psyche-ColorOS17-v1.0.zip` in Magisk or KernelSU.

## Verification & Debugging
After rebooting, run:
```sh
su -c '
service list | grep -i vib
dumpsys vibrator_manager | head -n 60
logcat -d -s vendor.vibratorfeature | tail -n 30
'
```
To enable verbose call tracing:
```sh
su -c 'mkdir -p /data/vendor/ohaptics && chmod 777 /data/vendor/ohaptics && stop vibratorfeature-hal-service && start vibratorfeature-hal-service'
```
If `EVIOCSFF` logs an error for `custom_id=197`, you can test another ID without recompiling:
```sh
su -c 'setprop persist.vendor.ohaptics.custom_wave_id <ID> && stop vibratorfeature-hal-service && start vibratorfeature-hal-service'
```
