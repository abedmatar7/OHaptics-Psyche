#!/system/bin/sh
# Wait briefly for servicemanager; restart vibratorfeature-hal-service if IVibrator/default is missing
for _ in 1 2 3 4 5 6 7 8 9 10; do
    if service check android.hardware.vibrator.IVibrator/default 2>/dev/null | grep -q "found"; then
        exit 0
    fi
    sleep 1
done
stop vibratorfeature-hal-service 2>/dev/null
sleep 1
start vibratorfeature-hal-service 2>/dev/null
