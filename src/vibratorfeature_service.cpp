// vendor.xiaomi.hardware.vibratorfeature.service adapted for Xiaomi 12X (psyche) on ColorOS 17.
// Adapted from https://github.com/nischay12089/Oplus-vibrator-feature
//
// Replaces Xiaomi's vibrator HAL binary at /vendor/bin/hw/vendor.xiaomi.hardware.vibratorfeature.service
// (systemlessly via Magisk/KernelSU) so Xiaomi's stock init rc starts it as uid system, group system input.
//
// Serves:
//   * android.hardware.vibrator.IVibrator/default (what ColorOS 17 VibratorManagerService binds to)
//   * android.hardware.vibrator.IVibrator/vibratorfeature (stock Xiaomi instance compatibility)
//   * vendor.oplus.hardware.oplusvibrator.IOplusVibrator (V2) as binder extension (AAC RichTap HE)
//
// Hardware backend: Xiaomi 12X (psyche) aw8697_haptic force-feedback device (/dev/input/event* "aw8697_haptic")
//   * FF_GAIN                                  amplitude for on()
//   * FF_CONSTANT + replay.length              on(): plain buzz
//   * FF_PERIODIC (FF_CUSTOM, id=197) + /sys/bus/i2c/drivers/aw8697_haptic/2-005a/custom_wave
//                                              effect library & RichTap HE PCM streaming @ 24 kHz
//   * f0 read from f0_value (1701 -> 170.1 Hz) without triggering a boot calibration buzz
//
// Waveforms are loaded from /vendor/firmware/ohaptics/<id>_<name>_RTP.bin (with fallback to /odm/firmware
// and /vendor/firmware).

#include <android/binder_ibinder.h>
#include <android/binder_parcel.h>
#include <android/binder_status.h>
#include <android/log.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/system_properties.h>
#include <time.h>
#include <unistd.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define TAG "vendor.vibratorfeature"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)

namespace {

// ---- interfaces ---------------------------------------------------------------------------------
constexpr char kVibDefaultName[] = "android.hardware.vibrator.IVibrator/default";
constexpr char kVibFeatureName[] = "android.hardware.vibrator.IVibrator/vibratorfeature";
constexpr char kVibDescriptor[] = "android.hardware.vibrator.IVibrator";
constexpr char kVibHash[] = "ea8742d6993e1a82917da38b9938e537aa7fcb54";        // android.hardware.vibrator V2
constexpr int32_t kVibVersion = 2;
constexpr char kOplusDescriptor[] = "vendor.oplus.hardware.oplusvibrator.IOplusVibrator";
constexpr char kOplusHash[] = "b103e6d812aa472b9c5b7deefd7986fa729ce1a8";      // IOplusVibrator V2
constexpr int32_t kOplusVersion = 2;
constexpr char kCbDescriptor[] = "vendor.aac.hardware.richtap.vibrator.IRichtapCallback";
constexpr char kVibCbDescriptor[] = "android.hardware.vibrator.IVibratorCallback";
constexpr char kFeatures[] = "vibrator_lmvibrator=1\nvibrator_qcom_lmvibrator=1\nvibrator_richctap=1\n";
// IVibrator.getCapabilities: ON_CALLBACK | PERFORM_CALLBACK | AMPLITUDE_CONTROL
constexpr int32_t kCapabilities = 1 | 2 | 4;

constexpr transaction_code_t GET_INTERFACE_HASH = 16777214, GET_INTERFACE_VERSION = 16777215;
// android.hardware.vibrator.IVibrator
constexpr transaction_code_t VIB_GET_CAPABILITIES = 1, VIB_OFF = 2, VIB_ON = 3, VIB_PERFORM = 4,
                               VIB_GET_SUPPORTED_EFFECTS = 5, VIB_SET_AMPLITUDE = 6;
// vendor.oplus.hardware.oplusvibrator.IOplusVibrator
enum : transaction_code_t {
    INIT = 1, SET_DYNAMIC_SCALE, SET_F0, STOP, SET_AMPLITUDE, PERFORM_HE_PARAM, OFF, ON, PERFORM,
    PERFORM_ENVELOPE, PERFORM_RTP, PERFORM_HE, SET_HAPTIC_PARAM, LM_OFF, LM_ON, START_CALIBRATE,
    GET_CALIBRATE_RESULTS, GET_STATUS, SET_VMAX, SET_MOTOR_OLD_TEST, GET_SUPPORT_FEATURES, WRITE_GUN_TYPE,
    WRITE_GUN_MODE, WRITE_BULLET_NUM, WRITE_HAPTIC_AUDIO, SET_TOUCH_STYLE, GET_TOUCH_STYLE, SET_POWER_ON_SWITCH,
};
constexpr transaction_code_t CB_ON_CALLBACK = 1;  // IRichtapCallback.onCallback(int) - oneway

constexpr int kSoftOffset = 1000;

// ---- motor: Xiaomi 12X (psyche) aw8697_haptic force-feedback device -----------------------------
constexpr char kInputDir[] = "/dev/input";
constexpr const char* kMotorNames[] = {"aw8697_haptic", "awinic_haptic", "si_haptic"};
constexpr char kSysPathProp[] = "ro.vendor.ohaptics.sys_path";
constexpr char kF0Prop[] = "ro.vendor.ohaptics.f0";
constexpr char kCustomWaveIdProp[] = "persist.vendor.ohaptics.custom_wave_id";
constexpr int kDefaultCustomWaveId = 197;
constexpr float kDefaultF0 = 170.1f;    // psyche measured f0_value = 1701 (170.1 Hz)
constexpr int kRate = 24000;            // PCM sample rate (Hz)
constexpr int kPeriod = 512;            // custom_wave period; final short write marks end-of-stream
constexpr int kChunk = 1024;            // bytes per write
constexpr int kDrainMs = 45;
constexpr int kBriefMs = 150;

constexpr const char* kFirmwareDirs[] = {
    "/vendor/firmware/ohaptics",
    "/odm/firmware",
    "/vendor/firmware",
};

constexpr char kStyleKey[] = "touch_style_vibration_intensity";  // 0 = Crisp, 1 = Gentle
constexpr char kSettingsPath[] = "/data/system/users/0/settings_system.xml";
constexpr char kIntensityKey[] = "touch_stepless_vibration_intensity";
constexpr int kIntensityMax = 2400;

constexpr float kOutputScale = 0.75f;
constexpr float kEffectScale = 1.45f;
constexpr int kKneeMaxMs = 60;
constexpr float kMotorQ = 8.f;
constexpr float kPatternScale = 0.5f;
constexpr float kHeTrim = 0.36f;
constexpr float kSoftTrim = 0.3f;
constexpr float kGentleEffectScale = 0.435f;
constexpr int kTickMs = 20;
constexpr float kDurTrimFloor = 1.0f;
constexpr float kDurTrimExp = 0.35f;
constexpr int kLongMs = 300;
constexpr float kLongGain = 0.8f;

int64_t nowMs() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

std::string prop(const char* key) {
    char buf[PROP_VALUE_MAX] = {};
    __system_property_get(key, buf);
    return buf;
}

bool fileExists(const std::string& p) {
    struct stat st;
    return stat(p.c_str(), &st) == 0;
}

int msOf(size_t samples) { return static_cast<int>(samples * 1000 / kRate); }

float kneeSamples(std::vector<int8_t>& pcm, float d) {
    if (d <= 1.f) return 1.f;
    const float norm = tanhf(d);
    for (int8_t& v : pcm) {
        float u = d * static_cast<float>(v) / 127.f;
        v = static_cast<int8_t>(clampi(static_cast<int>(lrintf(127.f * tanhf(u) / norm)), -127, 127));
    }
    return d;
}

float scaleSamples(std::vector<int8_t>& pcm, float g) {
    if (g < 0.f) g = 0.f;
    if (g > 1.f) {
        int peak = 0;
        for (int8_t v : pcm) peak = std::max(peak, std::abs(static_cast<int>(v)));
        if (peak > 0) g = std::min(g, 127.f / static_cast<float>(peak));
    }
    if (g == 1.f) return g;
    for (int8_t& v : pcm)
        v = static_cast<int8_t>(clampi(static_cast<int>(lrintf(static_cast<float>(v) * g)), -127, 127));
    return g;
}

float shapeEffect(std::vector<int8_t>& pcm, float want, float* applied) {
    *applied = scaleSamples(pcm, want);
    if (*applied > 0.f && want > *applied + 1e-3f && msOf(pcm.size()) <= kKneeMaxMs)
        return kneeSamples(pcm, want / *applied);
    return 1.f;
}

float excursionOf(const std::vector<int8_t>& pcm, float f0) {
    const float w = 2.f * 3.14159265f * f0;
    const float dt = 1.f / kRate;
    float v = 0.f, p = 0.f, peak = 0.f;
    for (size_t k = 0; k < pcm.size() + kRate / 40; ++k) {
        float u = k < pcm.size() ? static_cast<float>(pcm[k]) : 0.f;
        v += (u - (w / kMotorQ) * v - w * w * p) * dt;
        p += v * dt;
        peak = std::max(peak, fabsf(p));
    }
    return peak;
}

int abxInt(const std::string& xml, const char* key, int fallback) {
    size_t at = xml.find(key);
    if (at == std::string::npos) return fallback;
    size_t from = at + strlen(key);
    size_t left = xml.size() - from;
    const unsigned char* p = reinterpret_cast<const unsigned char*>(xml.data()) + from;
    if (left < 6 || p[0] != 0x2f) return fallback;
    size_t len = (static_cast<size_t>(p[3]) << 8) | p[4];
    if (len < 1 || len > 8 || left < 5 + len) return fallback;
    std::string digits(reinterpret_cast<const char*>(p + 5), len);
    if (digits.find_first_not_of("0123456789") != std::string::npos) return fallback;
    return atoi(digits.c_str());
}

struct Settings {
    int intensity = -1;
    int style = -1;
};

Settings readSettings() {
    static std::mutex mu;
    static int64_t checked = 0;
    static timespec seen = {0, 0};
    static Settings snap;
    std::lock_guard<std::mutex> l(mu);
    int64_t now = nowMs();
    if (checked != 0 && now - checked < 1000) return snap;
    checked = now;
    static bool warned = false;
    auto unreadable = [](const char* what) {
        if (warned) return;
        warned = true;
        LOGW("cannot %s %s: %s - intensity and touch style fall back to what the framework announces", what,
             kSettingsPath, strerror(errno));
    };
    struct stat st;
    if (stat(kSettingsPath, &st) != 0) {
        unreadable("stat");
        return snap;
    }
    if (st.st_mtim.tv_sec == seen.tv_sec && st.st_mtim.tv_nsec == seen.tv_nsec) return snap;
    seen = st.st_mtim;
    FILE* f = fopen(kSettingsPath, "re");
    if (f == nullptr) {
        unreadable("open");
        return snap;
    }
    std::string xml;
    char buf[8192];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) xml.append(buf, n);
    fclose(f);
    snap.intensity = abxInt(xml, kIntensityKey, snap.intensity);
    snap.style = abxInt(xml, kStyleKey, snap.style);
    return snap;
}

float userIntensity() {
    int v = readSettings().intensity;
    return (v >= 0 && v <= kIntensityMax) ? static_cast<float>(v) / kIntensityMax : 1.f;
}

void traceLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

class Motor {
public:
    bool start() {
        if (!openDevice()) return false;
        sysfs_ = findSysfsPath();
        wave_ = sysfs_ + "/custom_wave";
        std::string idStr = prop(kCustomWaveIdProp);
        if (!idStr.empty()) {
            int parsed = atoi(idStr.c_str());
            if (parsed > 0 && parsed <= 32767) customWaveId_ = static_cast<int16_t>(parsed);
        }
        f0_ = readF0();
        std::thread([this] { streamLoop(); }).detach();
        LOGI("motor: %s (%s), custom_wave %s (id=%d), f0 %.1f Hz",
             dev_.c_str(), motorName_.c_str(), wave_.c_str(),
             static_cast<int>(customWaveId_), static_cast<double>(f0_));
        return true;
    }

    bool ready() const { return fd_ >= 0; }
    float f0() const { return f0_; }
    bool streaming() const { return streamId_.load() >= 0; }

    void setGain(float g) {
        if (fd_ < 0) return;
        int32_t v = clampi(static_cast<int>(lrintf(g * kOutputScale * 0x7fff)), 0, 0x7fff);
        std::lock_guard<std::mutex> l(evMu_);
        if (v == gain_) return;
        gain_ = v;
        sendEvent(FF_GAIN, v);
    }

    void buzz(int ms) {
        traceLog("  motor.buzz ms=%d", ms);
        stop(true);
        playEnd_.store(nowMs() + ms + kDrainMs);
        looping_.store(false);
        if (fd_ < 0 || ms <= 0) return;
        std::lock_guard<std::mutex> l(evMu_);
        ff_effect e = {};
        e.type = FF_CONSTANT;
        e.replay.length = static_cast<uint16_t>(std::min(ms, 0xffff));
        e.u.constant.level = 0x7fff;
        int id = upload(e);
        if (id < 0) return;
        sendEvent(static_cast<uint16_t>(id), 1);
        buzzId_ = id;
    }

    void play(std::vector<int8_t> pcm, int loops, int gapMs) {
        traceLog("  motor.play bytes=%zu loops=%d gap=%d fd=%d", pcm.size(), loops, gapMs, fd_);
        if (fd_ < 0 || pcm.empty()) return;
        {
            std::lock_guard<std::mutex> l(evMu_);
            if (gain_ != 0x7fff) {
                gain_ = 0x7fff;
                sendEvent(FF_GAIN, 0x7fff);
            }
        }
        int64_t len = msOf(pcm.size());
        looping_.store(loops < 0);
        playEnd_.store(loops < 0 ? INT64_MAX / 4
                                 : nowMs() + loops * len + std::max(0, loops - 1) * gapMs + kDrainMs);
        {
            std::lock_guard<std::mutex> l(mu_);
            pcm_ = std::move(pcm);
            loops_ = loops;
            gap_ = std::max(0, gapMs);
            gen_++;
            work_ = true;
        }
        abortPlayback();
        cv_.notify_all();
    }

    void stop(bool force = false) {
        int64_t left = playEnd_.load() - nowMs();
        if (!force && !looping_.load() && left > 0 && left <= kBriefMs) {
            traceLog("  motor.stop ignored, %lldms of a brief effect left", static_cast<long long>(left));
            return;
        }
        traceLog("  motor.stop");
        playEnd_.store(0);
        {
            std::lock_guard<std::mutex> l(mu_);
            gen_++;
            work_ = false;
        }
        {
            std::lock_guard<std::mutex> l(evMu_);
            if (buzzId_ >= 0) {
                sendEvent(static_cast<uint16_t>(buzzId_), 0);
                erase(buzzId_);
                buzzId_ = -1;
            }
        }
        abortPlayback();
    }

private:
    std::string findSysfsPath() {
        std::string p = prop(kSysPathProp);
        if (!p.empty() && fileExists(p + "/custom_wave")) return p;
        // Primary Xiaomi 12X (psyche) path
        const char* candidates[] = {
            "/sys/bus/i2c/drivers/aw8697_haptic/2-005a",
            "/sys/bus/i2c/drivers/aw8697_haptic/3-005a",
            "/sys/bus/i2c/drivers/aw8697_haptic/0-005a",
            "/sys/bus/i2c/drivers/aw8697_haptic/1-005a",
            "/sys/bus/i2c/drivers/awinic_haptic/2-005a",
            "/sys/bus/i2c/drivers/awinic_haptic/3-005a",
        };
        for (const char* c : candidates) {
            if (fileExists(std::string(c) + "/custom_wave")) return c;
        }
        return "/sys/bus/i2c/drivers/aw8697_haptic/2-005a";
    }

    void abortPlayback() {
        int id = streamId_.exchange(-1);
        if (id < 0) return;
        std::lock_guard<std::mutex> l(evMu_);
        sendEvent(static_cast<uint16_t>(id), 0);
        erase(id);
    }

    bool openDevice() {
        DIR* d = opendir(kInputDir);
        if (d == nullptr) return false;
        while (dirent* e = readdir(d)) {
            if (strncmp(e->d_name, "event", 5) != 0) continue;
            std::string path = std::string(kInputDir) + "/" + e->d_name;
            int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
            if (fd < 0) continue;
            char name[128] = {};
            unsigned long bits[4] = {};
            bool nameOk = false;
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0) {
                for (const char* target : kMotorNames) {
                    if (strcmp(name, target) == 0) {
                        nameOk = true;
                        break;
                    }
                }
            }
            bool ffOk = nameOk && ioctl(fd, EVIOCGBIT(EV_FF, sizeof(bits)), bits) >= 0 &&
                        (bits[FF_CUSTOM / (8 * sizeof(long))] & (1UL << (FF_CUSTOM % (8 * sizeof(long)))));
            if (ffOk) {
                fd_ = fd;
                dev_ = path;
                motorName_ = name;
                break;
            }
            close(fd);
        }
        closedir(d);
        if (fd_ < 0) LOGE("no aw8697_haptic/awinic_haptic force-feedback device under %s", kInputDir);
        return fd_ >= 0;
    }

    float readF0() {
        // Prefer f0_value (read-only cached calibration on aw8697) so reading it does not buzz at boot.
        const char* nodes[] = {"/f0_value", "/f0_save", "/f0"};
        for (const char* node : nodes) {
            FILE* f = fopen((sysfs_ + node).c_str(), "re");
            if (f == nullptr) continue;
            int raw = 0;
            if (fscanf(f, "%d", &raw) != 1) raw = 0;
            fclose(f);
            if (raw > 1000 && raw < 5000) return raw / 10.0f;
            if (raw > 100 && raw < 500) return static_cast<float>(raw);
        }
        float p = static_cast<float>(atof(prop(kF0Prop).c_str()));
        return p > 50.f ? p : kDefaultF0;
    }

    bool sendEvent(uint16_t code, int32_t value) {
        input_event e = {};
        e.type = EV_FF;
        e.code = code;
        e.value = value;
        if (write(fd_, &e, sizeof(e)) == static_cast<ssize_t>(sizeof(e))) return true;
        LOGW("EV_FF code %u value %d: %s", code, value, strerror(errno));
        return false;
    }

    int upload(ff_effect& e) {
        e.id = -1;
        if (ioctl(fd_, EVIOCSFF, &e) >= 0) return e.id;
        LOGW("EVIOCSFF (type=%u custom_id=%d): %s",
             e.type, e.type == FF_PERIODIC ? static_cast<int>(customWaveId_) : -1, strerror(errno));
        return -1;
    }

    void erase(int id) {
        if (id >= 0) ioctl(fd_, EVIOCRMFF, id);
    }

    bool beginStream() {
        ff_effect e = {};
        e.type = FF_PERIODIC;
        e.u.periodic.waveform = FF_CUSTOM;
        e.u.periodic.magnitude = 0x7fff;
        int16_t data[3] = {customWaveId_, 0, 0};
        e.u.periodic.custom_len = sizeof(data);
        e.u.periodic.custom_data = data;
        std::lock_guard<std::mutex> l(evMu_);
        int id = upload(e);
        if (id < 0) return false;
        waveFd_ = ::open(wave_.c_str(), O_WRONLY | O_CLOEXEC);
        if (waveFd_ < 0) {
            LOGW("open %s: %s", wave_.c_str(), strerror(errno));
            erase(id);
            return false;
        }
        if (!sendEvent(static_cast<uint16_t>(id), 1)) {
            close(waveFd_);
            waveFd_ = -1;
            erase(id);
            return false;
        }
        streamId_ = id;
        return true;
    }

    void endStream() {
        if (waveFd_ >= 0) {
            close(waveFd_);
            waveFd_ = -1;
        }
        abortPlayback();
    }

    bool streamOnce(const std::vector<int8_t>& pcm, uint64_t gen) {
        if (!beginStream()) return true;
        size_t off = 0, whole = pcm.size() - pcm.size() % kPeriod;
        while (off < whole) {
            if (superseded(gen)) return false;
            if (waveFd_ < 0) return true;
            size_t n = std::min(whole - off, static_cast<size_t>(kChunk));
            ssize_t w = write(waveFd_, pcm.data() + off, n);
            if (w > 0) {
                off += static_cast<size_t>(w);
            } else if (w == 0 || errno == EINTR || errno == EAGAIN || errno == ENOSPC) {
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
            } else {
                LOGW("custom_wave write %zu: %s", n, strerror(errno));
                return !superseded(gen);
            }
        }
        if (superseded(gen)) return false;
        int8_t tail[kPeriod] = {};
        size_t rest = pcm.size() - whole;
        if (rest > 0) memcpy(tail, pcm.data() + whole, rest);
        else rest = 1;
        if (waveFd_ >= 0 && write(waveFd_, tail, rest) < 0 && errno != EINTR)
            LOGW("custom_wave tail %zu: %s", rest, strerror(errno));
        return true;
    }

    bool waitUntil(int64_t until, uint64_t gen) {
        while (nowMs() < until) {
            if (superseded(gen)) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return !superseded(gen);
    }

    bool superseded(uint64_t gen) {
        std::lock_guard<std::mutex> l(mu_);
        return gen_ != gen;
    }

    void streamLoop() {
        for (;;) {
            std::vector<int8_t> pcm;
            int loops, gap;
            uint64_t gen;
            {
                std::unique_lock<std::mutex> l(mu_);
                cv_.wait(l, [this] { return work_ || quit_; });
                if (quit_) return;
                work_ = false;
                pcm = pcm_;
                loops = loops_;
                gap = gap_;
                gen = gen_;
            }
            endStream();
            for (int i = 0; loops < 0 || i < loops; i++) {
                int64_t t0 = nowMs();
                if (!streamOnce(pcm, gen)) break;
                if (!waitUntil(t0 + msOf(pcm.size()) + kDrainMs, gen)) break;
                if (loops >= 0 && i + 1 >= loops) break;
                if (gap > 0 && !waitUntil(nowMs() + gap, gen)) break;
                endStream();
            }
            endStream();
        }
    }

    int fd_ = -1, waveFd_ = -1, buzzId_ = -1;
    int16_t customWaveId_ = kDefaultCustomWaveId;
    std::atomic<int> streamId_{-1};
    int32_t gain_ = -1;
    std::atomic<int64_t> playEnd_{0};
    std::atomic<bool> looping_{false};
    std::string dev_, motorName_, sysfs_, wave_;
    float f0_ = kDefaultF0;
    std::mutex evMu_;
    std::mutex mu_;
    std::condition_variable cv_;
    bool work_ = false, quit_ = false;
    uint64_t gen_ = 0;
    std::vector<int8_t> pcm_;
    int loops_ = 1, gap_ = 0;
};
Motor gMotor;

// ---- effect library: <id>_<name>_RTP.bin, raw int8 PCM at 24 kHz --------------------------------
class Library {
public:
    void scan() {
        for (const char* dir : kFirmwareDirs) {
            DIR* d = opendir(dir);
            if (d == nullptr) continue;
            size_t added = 0;
            while (dirent* e = readdir(d)) {
                int id = -1;
                char rest[256] = {};
                if (sscanf(e->d_name, "%d_%255s", &id, rest) != 2 || id < 0) continue;
                const char* dot = strstr(rest, "_RTP.bin");
                if (dot == nullptr || dot[8] != '\0') continue;
                if (files_.count(id) == 0) {
                    files_[id] = std::string(dir) + "/" + e->d_name;
                    added++;
                }
            }
            closedir(d);
            if (added > 0) LOGI("effect library: +%zu waveforms from %s", added, dir);
        }
        LOGI("effect library: %zu total waveforms", files_.size());
    }

    bool has(int id) const { return files_.count(id) != 0; }

    std::vector<int8_t> load(int id) const {
        std::vector<int8_t> pcm;
        auto it = files_.find(id);
        if (it == files_.end()) return pcm;
        FILE* f = fopen(it->second.c_str(), "re");
        if (f == nullptr) return pcm;
        char buf[8192];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), f)) > 0) pcm.insert(pcm.end(), buf, buf + n);
        fclose(f);
        return pcm;
    }

    float headroom() const { return headroom_; }
    float softRatio() const { return softRatio_; }
    float excursionCeiling() const { return excursionCeiling_; }

    void measureExcursion(float f0) {
        for (const auto& e : files_) {
            struct stat st;
            if (e.first >= kSoftOffset || stat(e.second.c_str(), &st) != 0) continue;
            if (st.st_size <= 0 || msOf(static_cast<size_t>(st.st_size)) > kTickMs) continue;
            excursionCeiling_ = std::max(excursionCeiling_, excursionOf(load(e.first), f0));
        }
        LOGI("excursion ceiling: %.6f at f0 %.1f Hz", static_cast<double>(excursionCeiling_),
             static_cast<double>(f0));
    }

    int peakOf(int id) const {
        int peak = 0;
        for (int8_t v : load(id)) peak = std::max(peak, std::abs(static_cast<int>(v)));
        return peak;
    }

    void measureHeadroom() {
        std::vector<int> peaks;
        for (int id : {0, 1, 2, 3, 4, 6, 7}) {
            int peak = peakOf(id);
            if (peak > 0) peaks.push_back(peak);
        }
        if (!peaks.empty()) {
            std::sort(peaks.begin(), peaks.end());
            headroom_ = clampi(peaks[peaks.size() / 2], 40, 127) / 127.f;
            LOGI("library headroom: %d of 127 (%.2f) from %zu short effects", peaks[peaks.size() / 2],
                 static_cast<double>(headroom_), peaks.size());
        }
        std::vector<float> ratios;
        for (const auto& e : files_) {
            if (e.first >= kSoftOffset || !has(e.first + kSoftOffset)) continue;
            int base = peakOf(e.first), soft = peakOf(e.first + kSoftOffset);
            if (base > 0 && soft > 0) ratios.push_back(static_cast<float>(soft) / base);
        }
        if (!ratios.empty()) {
            std::sort(ratios.begin(), ratios.end());
            float r = ratios[ratios.size() / 2];
            softRatio_ = r < 0.2f ? 0.2f : (r > 1.f ? 1.f : r);
            LOGI("library soft ratio: %.2f from %zu soft/normal pairs", static_cast<double>(softRatio_),
                 ratios.size());
        }
    }

    std::vector<int32_t> ids() const {
        std::vector<int32_t> v;
        for (const auto& e : files_)
            if (e.first < kSoftOffset) v.push_back(e.first);
        return v;
    }

private:
    std::map<int, std::string> files_;
    float headroom_ = 1.f;
    float softRatio_ = 1.f;
    float excursionCeiling_ = 0.f;
};
Library gLibrary;

AIBinder_Class* gVibClass = nullptr;
AIBinder_Class* gOplusClass = nullptr;
AIBinder_Class* gCbClass = nullptr;
AIBinder_Class* gVibCbClass = nullptr;
AIBinder* gSelfDefault = nullptr;
AIBinder* gSelfFeature = nullptr;

struct Vndk {
    binder_exception_t (*addService)(AIBinder*, const char*);
    bool (*setThreadPoolMaxThreadCount)(uint32_t);
    void (*startThreadPool)();
    void (*joinThreadPool)();
    void (*markVintfStability)(AIBinder*);
} gV;

template <typename T>
bool resolve(void* lib, const char* name, T& out) {
    out = reinterpret_cast<T>(dlsym(lib, name));
    if (out == nullptr) LOGE("missing %s in libbinder_ndk", name);
    return out != nullptr;
}

bool loadVndk() {
    void* lib = dlopen("libbinder_ndk.so", RTLD_NOW);
    if (lib == nullptr) return false;
    return resolve(lib, "AServiceManager_addService", gV.addService) &&
           resolve(lib, "ABinderProcess_setThreadPoolMaxThreadCount", gV.setThreadPoolMaxThreadCount) &&
           resolve(lib, "ABinderProcess_startThreadPool", gV.startThreadPool) &&
           resolve(lib, "ABinderProcess_joinThreadPool", gV.joinThreadPool) &&
           resolve(lib, "AIBinder_markVintfStability", gV.markVintfStability);
}

constexpr char kTracePath[] = "/data/vendor/ohaptics/trace.log";
void traceLog(const char* fmt, ...) {
    static int fd = ::open(kTracePath, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0666);
    if (fd < 0) return;
    char buf[320];
    int head = snprintf(buf, sizeof(buf), "%lld ", static_cast<long long>(nowMs()));
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + head, sizeof(buf) - head - 2, fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    size_t len = static_cast<size_t>(head + n);
    buf[len++] = '\n';
    static std::mutex mu;
    std::lock_guard<std::mutex> l(mu);
    if (write(fd, buf, len) < 0) return;
}

binder_status_t ok(AParcel* out) {
    AStatus* st = AStatus_newOk();
    binder_status_t r = AParcel_writeStatusHeader(out, st);
    AStatus_delete(st);
    return r;
}

binder_status_t exception(AParcel* out, binder_exception_t ex) {
    AStatus* st = AStatus_fromExceptionCode(ex);
    binder_status_t r = AParcel_writeStatusHeader(out, st);
    AStatus_delete(st);
    return r;
}

bool softStyleActive();

void fireCallback(AIBinder* cb) {
    bool vib = AIBinder_associateClass(cb, gVibCbClass);
    bool richtap = !vib && AIBinder_associateClass(cb, gCbClass);
    if (!vib && !richtap) {
        LOGW("completion callback is neither %s nor %s", kCbDescriptor, kVibCbDescriptor);
        return;
    }
    AParcel* in = nullptr;
    if (AIBinder_prepareTransaction(cb, &in) != STATUS_OK) return;
    if (richtap) AParcel_writeInt32(in, 0);
    AParcel* reply = nullptr;
    AIBinder_transact(cb, CB_ON_CALLBACK, &in, &reply, FLAG_ONEWAY);
    AParcel_delete(reply);
}

class Timer {
public:
    void at(int64_t when, AIBinder* cb) {
        if (cb == nullptr) return;
        AIBinder_incStrong(cb);
        std::lock_guard<std::mutex> g(mu_);
        if (!started_) {
            std::thread([this] { run(); }).detach();
            started_ = true;
        }
        q_.emplace(when, cb);
        cv_.notify_one();
    }
    void flush() {
        std::lock_guard<std::mutex> g(mu_);
        std::multimap<int64_t, AIBinder*> now;
        for (auto& e : q_) now.emplace(0, e.second);
        q_.swap(now);
        cv_.notify_one();
    }

private:
    void run() {
        std::unique_lock<std::mutex> l(mu_);
        for (;;) {
            if (q_.empty()) {
                cv_.wait(l);
                continue;
            }
            int64_t wait = q_.begin()->first - nowMs();
            if (wait > 0) {
                cv_.wait_for(l, std::chrono::milliseconds(wait));
                continue;
            }
            AIBinder* cb = q_.begin()->second;
            q_.erase(q_.begin());
            l.unlock();
            fireCallback(cb);
            AIBinder_decStrong(cb);
            l.lock();
        }
    }
    std::mutex mu_;
    std::condition_variable cv_;
    std::multimap<int64_t, AIBinder*> q_;
    bool started_ = false;
};
Timer gTimer;

// ---- RichTap HE -> one PCM timeline -------------------------------------------------------------
constexpr int kContinuous = 4096, kTransient = 4097;
constexpr int kMaxEventMs = 5000;
constexpr int kTransientMs = 48;
constexpr int kMaxTimelineMs = 30000;

struct Ev {
    bool cont = false;
    int t = 0, inten = 0, freq = 50, dur = 0;
    std::vector<int> pt, pi, pf;
    int end() const { return t + (cont ? dur : kTransientMs); }
};

std::mutex gLock;
int gAmp = 255, gInterval = 0, gFreq = 0;
int gPid = -1, gSeq = -1, gTotal = 0, gGot = 0;
std::vector<Ev> gPending;
int64_t gStart = 0;

void readPoints(Ev& e, const std::vector<int32_t>& he, size_t o, int n) {
    for (int i = 0; i < n; i++) {
        e.pt.push_back(clampi(he[o + 3 * i], 0, e.dur));
        e.pi.push_back(clampi(he[o + 3 * i + 1], 0, 100));
        e.pf.push_back(clampi(he[o + 3 * i + 2], -100, 100));
    }
}

void parseHe1(const std::vector<int32_t>& he, std::vector<Ev>& out) {
    for (size_t b = 0; b + 7 <= he.size(); b += 55) {
        int type = he[b];
        if (type != kContinuous && type != kTransient) break;
        Ev e;
        e.t = std::max(0, he[b + 1]);
        e.inten = clampi(he[b + 2], 0, 100);
        e.freq = clampi(he[b + 3], 0, 100);
        if (type == kContinuous) {
            e.cont = true;
            e.dur = clampi(he[b + 4], 0, kMaxEventMs);
            int n = std::min(clampi(he[b + 6], 0, 16), static_cast<int>((he.size() - b - 7) / 3));
            readPoints(e, he, b + 7, n);
        }
        out.push_back(e);
    }
}

void parseHe2(const std::vector<int32_t>& he, std::vector<Ev>& out) {
    int patterns = static_cast<uint32_t>(he[4]) >> 16;
    size_t o = 5;
    for (int p = 0; p < patterns && o + 3 <= he.size(); p++) {
        int abs = he[o + 1], nEv = he[o + 2];
        o += 3;
        for (int k = 0; k < nEv && o + 7 <= he.size(); k++) {
            int type = he[o], len = he[o + 1];
            Ev e;
            e.t = std::max(0, abs + he[o + 3]);
            e.inten = clampi(he[o + 4], 0, 100);
            e.freq = clampi(he[o + 5], 0, 100);
            if (type == kContinuous && o + 8 <= he.size()) {
                e.cont = true;
                e.dur = clampi(he[o + 6], 0, kMaxEventMs);
                int n = std::min(clampi(he[o + 7], 0, 16), static_cast<int>((he.size() - o - 8) / 3));
                readPoints(e, he, o + 8, n);
            }
            if (type == kContinuous || type == kTransient) out.push_back(e);
            o += std::max(5, len) + 2;
        }
    }
}

int patternEnd(const std::vector<Ev>& evs) {
    int end = 0;
    for (const Ev& e : evs) end = std::max(end, e.end());
    return end;
}

float heLevel(int inten) { return clampi(inten, 0, 100) / 100.f; }

float curveAt(const Ev& e, float ms) {
    if (e.pt.size() < 2) return 1.f;
    if (ms <= e.pt.front()) return e.pi.front() / 100.f;
    for (size_t i = 0; i + 1 < e.pt.size(); i++) {
        if (ms > e.pt[i + 1]) continue;
        float span = static_cast<float>(e.pt[i + 1] - e.pt[i]);
        float k = span > 0 ? (ms - e.pt[i]) / span : 0.f;
        return (e.pi[i] + k * (e.pi[i + 1] - e.pi[i])) / 100.f;
    }
    return e.pi.back() / 100.f;
}

float carrierHz(int freq) {
    (void)freq;
    return gMotor.f0();
}

std::vector<int8_t> render(const std::vector<Ev>& evs, int shiftMs) {
    int end = std::min(patternEnd(evs) - shiftMs, kMaxTimelineMs);
    if (end <= 0 || evs.empty()) return {};
    std::vector<float> env(static_cast<size_t>(kRate) * end / 1000 + 1, 0.f);
    auto samples = [](int ms) { return static_cast<size_t>(static_cast<int64_t>(kRate) * ms / 1000); };
    for (const Ev& e : evs) {
        int startMs = e.t - shiftMs;
        int lenMs = e.cont ? e.dur : kTransientMs;
        if (lenMs <= 0 || startMs + lenMs <= 0) continue;
        float level = heLevel(e.inten);
        if (e.cont && lenMs > kLongMs) level *= kLongGain;
        size_t skip = startMs < 0 ? samples(-startMs) : 0;
        size_t base = startMs > 0 ? samples(startMs) : 0;
        size_t n = samples(lenMs);
        for (size_t i = skip; i < n; i++) {
            size_t at = base + (i - skip);
            if (at >= env.size()) break;
            float ms = i * 1000.f / kRate;
            float a;
            if (e.cont) {
                a = curveAt(e, ms);
                a *= std::min(1.f, ms / 4.f);
                a *= std::max(0.f, std::min(1.f, (lenMs - ms) / 6.f));
            } else {
                a = expf(-4.5f * ms / lenMs) * std::min(1.f, ms / 1.5f);
            }
            env[at] += level * a;
        }
    }
    float peak = 0.f;
    for (float v : env) peak = std::max(peak, v);
    float target = gLibrary.headroom() * kHeTrim;
    float norm = (peak > target && peak > 0.f) ? target / peak : 1.f;
    const float w = 2.f * static_cast<float>(M_PI) * carrierHz(50) / kRate;
    std::vector<int8_t> pcm(env.size());
    for (size_t i = 0; i < pcm.size(); i++) {
        float v = env[i] * norm * 127.f * sinf(w * static_cast<float>(i));
        pcm[i] = static_cast<int8_t>(clampi(static_cast<int>(lrintf(v)), -127, 127));
    }
    traceLog("  render %zu events %dms peak %.2f -> %.2f", evs.size(), end, static_cast<double>(peak),
             static_cast<double>(peak * norm));
    return pcm;
}

int64_t playTimeline(const std::vector<Ev>& evs, int shift, int loops, int interval, int amp) {
    if (evs.empty() || amp <= 0) return 0;
    std::vector<int8_t> pcm = render(evs, shift);
    if (pcm.empty()) return 0;
    float scale = clampi(amp, 0, 255) / 255.f * userIntensity();
    bool softNow = softStyleActive();
    if (softNow) scale *= gLibrary.softRatio() * kSoftTrim;
    traceLog("  he gain=%.2f (amp %d x intensity %.2f soft=%d) -> samples x%.3f", static_cast<double>(scale),
             amp, static_cast<double>(userIntensity()), softNow ? 1 : 0,
             static_cast<double>(scale * kPatternScale));
    scaleSamples(pcm, scale * kPatternScale);
    gMotor.play(pcm, loops, interval);
    int64_t len = msOf(pcm.size());
    if (loops < 0) return INT64_MAX / 4;
    return static_cast<int64_t>(loops) * len + static_cast<int64_t>(loops - 1) * interval;
}

void stopAll() {
    {
        std::lock_guard<std::mutex> g(gLock);
        gPid = gSeq = -1;
        gPending.clear();
    }
    gMotor.stop();
    gTimer.flush();
}

void performHe(int looper, int interval, int amplitude, int freq, const std::vector<int32_t>& he, AIBinder* cb) {
    int loops = looper == -1 ? -1 : (looper < 0 ? 0 : std::max(1, looper));
    int64_t now = nowMs();
    if (he.empty() || loops == 0) {
        gTimer.at(now, cb);
        return;
    }
    std::vector<Ev> raw;
    if (he[0] == 2) parseHe2(he, raw);
    else parseHe1(he, raw);

    int amp, ival, shift = 0;
    std::vector<Ev> evs;
    {
        std::lock_guard<std::mutex> g(gLock);
        gAmp = amp = amplitude > 0 ? clampi(amplitude, 0, 255) : 255;
        gInterval = ival = std::max(0, interval);
        gFreq = freq;
        if (he[0] == 2 && he.size() >= 5) {
            int pid = he[2], seq = he[3], total = he[4] & 0xffff;
            if (pid != gPid || seq != gSeq || gGot >= gTotal) {
                gPid = pid;
                gSeq = seq;
                gTotal = std::max(1, total);
                gGot = 0;
                gStart = now;
                gPending.clear();
            }
            gGot += std::max(1, static_cast<int>(static_cast<uint32_t>(he[4]) >> 16));
            shift = static_cast<int>(now - gStart);
            gPending.insert(gPending.end(), raw.begin(), raw.end());
            evs = gPending;
            if (gTotal > 1 && loops != 1) loops = 1;
        } else {
            gPid = gSeq = -1;
            gPending.clear();
            evs = raw;
        }
    }
    std::stable_sort(evs.begin(), evs.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
    int64_t len = playTimeline(evs, shift, loops, ival, amp);
    if (loops >= 0) gTimer.at(now + len, cb);
    traceLog("  he%d events=%zu->%zu loops=%d amp=%d(asked %d) len=%lldms", he[0] == 2 ? 2 : 1,
             raw.size(), evs.size(), loops, amp, amplitude, static_cast<long long>(len));
}

void playOnce(std::vector<Ev> evs, AIBinder* cb) {
    int amp;
    {
        std::lock_guard<std::mutex> g(gLock);
        amp = gAmp;
    }
    int64_t now = nowMs();
    std::stable_sort(evs.begin(), evs.end(), [](const Ev& a, const Ev& b) { return a.t < b.t; });
    int64_t len = playTimeline(evs, 0, 1, 0, amp);
    gTimer.at(now + len, cb);
}

// ---- OnePlus calling convention on IVibrator ----------------------------------------------------
float durationTrim(int ms) {
    if (ms <= kTickMs) return 1.f;
    float t = powf(static_cast<float>(kTickMs) / static_cast<float>(ms), kDurTrimExp);
    return t < kDurTrimFloor ? kDurTrimFloor : t;
}

std::mutex gStyleLock;
float gPendingGain = -1.f;
float gAmplitude = 1.f;
bool gSoftStyle = false;
int gStyleSeen = -1;

bool softStyleActive() {
    int stored = readSettings().style;
    std::lock_guard<std::mutex> g(gStyleLock);
    if (stored >= 0 && stored != gStyleSeen) {
        gStyleSeen = stored;
        gSoftStyle = stored == 1;
    }
    return gSoftStyle;
}

binder_status_t vibSetAmplitude(const AParcel* in, AParcel* out) {
    float a = 0;
    AParcel_readFloat(in, &a);
    traceLog("  setAmplitude %.4f", static_cast<double>(a));
    std::lock_guard<std::mutex> g(gStyleLock);
    if (a <= 1.0f) {
        gAmplitude = a < 0.f ? 0.f : a;
        if (!gMotor.streaming()) gMotor.setGain(gAmplitude);
    } else if (a == 10000.f || a == 10001.f) {
        gSoftStyle = a == 10001.f;
    } else if (a <= 2.0f) {
        gPendingGain = a - 1.0f;
    } else {
        gPendingGain = 1.0f;
    }
    return ok(out);
}

binder_status_t vibPerform(const AParcel* in, AParcel* out) {
    int32_t effect = 0;
    int8_t strength = 0;
    AIBinder* cb = nullptr;
    AParcel_readInt32(in, &effect);
    AParcel_readByte(in, &strength);
    AParcel_readStrongBinder(in, &cb);
    bool soft = softStyleActive();
    float gain;
    {
        std::lock_guard<std::mutex> g(gStyleLock);
        gain = gPendingGain;
        gPendingGain = -1.f;
        if (gain < 0.f) gain = gAmplitude;
    }
    int id = effect;
    bool playedSoft = soft && gLibrary.has(id + kSoftOffset);
    if (playedSoft) id += kSoftOffset;
    else if (soft) gain *= gLibrary.softRatio();
    std::vector<int8_t> pcm = gLibrary.load(id);
    traceLog("  perform effect=%d -> id=%d strength=%d gain=%.2f soft=%d hasSoft=%d bytes=%zu", effect, id,
             static_cast<int>(strength), static_cast<double>(gain), soft ? 1 : 0,
             playedSoft ? 1 : 0, pcm.size());
    if (pcm.empty()) {
        LOGE("no waveform for effect %d", id);
        if (cb) AIBinder_decStrong(cb);
        return exception(out, EX_UNSUPPORTED_OPERATION);
    }
    int ms = msOf(pcm.size());
    gain *= durationTrim(ms);
    float want = gain * (soft ? kGentleEffectScale : kEffectScale);
    const std::vector<int8_t> authored = pcm;
    float applied = 0.f;
    float knee = shapeEffect(pcm, want, &applied);
    float ceiling = gLibrary.excursionCeiling();
    float limited = want;
    if (want > 1.f && ms <= kKneeMaxMs && ceiling > 0.f && excursionOf(pcm, gMotor.f0()) > ceiling) {
        float lo = 1.f, hi = want;
        for (int i = 0; i < 8; ++i) {
            float mid = 0.5f * (lo + hi);
            std::vector<int8_t> trial = authored;
            float a = 0.f;
            shapeEffect(trial, mid, &a);
            if (excursionOf(trial, gMotor.f0()) > ceiling) hi = mid;
            else lo = mid;
        }
        limited = lo;
        pcm = authored;
        knee = shapeEffect(pcm, limited, &applied);
    }
    traceLog("    knee x%.2f (%d ms), boost x%.3f of x%.3f asked", static_cast<double>(knee), ms,
             static_cast<double>(limited), static_cast<double>(want));
    gMotor.play(pcm, 1, 0);
    gTimer.at(nowMs() + ms, cb);
    if (cb) AIBinder_decStrong(cb);
    ok(out);
    return AParcel_writeInt32(out, ms);
}

binder_status_t onVibTransact(AIBinder*, transaction_code_t code, const AParcel* in, AParcel* out) {
    traceLog("IVibrator tx=%u", code);
    switch (code) {
        case VIB_GET_CAPABILITIES:
            ok(out);
            return AParcel_writeInt32(out, kCapabilities);
        case VIB_OFF:
            stopAll();
            return ok(out);
        case VIB_ON: {
            int32_t ms = 0;
            AIBinder* cb = nullptr;
            AParcel_readInt32(in, &ms);
            AParcel_readStrongBinder(in, &cb);
            {
                std::lock_guard<std::mutex> g(gStyleLock);
                gMotor.setGain(gAmplitude);
            }
            gMotor.buzz(ms);
            gTimer.at(nowMs() + ms, cb);
            if (cb) AIBinder_decStrong(cb);
            return ok(out);
        }
        case VIB_PERFORM:
            return vibPerform(in, out);
        case VIB_GET_SUPPORTED_EFFECTS: {
            std::vector<int32_t> ids = gLibrary.ids();
            ok(out);
            return AParcel_writeInt32Array(out, ids.data(), static_cast<int32_t>(ids.size()));
        }
        case VIB_SET_AMPLITUDE:
            return vibSetAmplitude(in, out);
        case GET_INTERFACE_VERSION:
            ok(out);
            return AParcel_writeInt32(out, kVibVersion);
        case GET_INTERFACE_HASH:
            ok(out);
            return AParcel_writeString(out, kVibHash, static_cast<int32_t>(strlen(kVibHash)));
        default:
            return exception(out, EX_UNSUPPORTED_OPERATION);
    }
}

// ---- IOplusVibrator -----------------------------------------------------------------------------
bool allocInts(void* data, int32_t length, int32_t** buffer) {
    auto* v = static_cast<std::vector<int32_t>*>(data);
    if (length < 0) {
        v->clear();
        *buffer = nullptr;
        return true;
    }
    v->resize(length);
    *buffer = v->data();
    return true;
}

AIBinder* readCb(const AParcel* in) {
    AIBinder* cb = nullptr;
    AParcel_readStrongBinder(in, &cb);
    return cb;
}

void release(AIBinder* cb) {
    if (cb) AIBinder_decStrong(cb);
}

binder_status_t onOplusTransact(AIBinder*, transaction_code_t code, const AParcel* in, AParcel* out) {
    traceLog("IOplusVibrator tx=%u", code);
    int32_t a = 0, b = 0, c = 0, d = 0;
    std::vector<int32_t> arr;
    AIBinder* cb = nullptr;
    switch (code) {
        case PERFORM_HE:
            AParcel_readInt32(in, &a);
            AParcel_readInt32(in, &b);
            AParcel_readInt32(in, &c);
            AParcel_readInt32(in, &d);
            AParcel_readInt32Array(in, &arr, allocInts);
            cb = readCb(in);
            performHe(a, b, c, d, arr, cb);
            release(cb);
            return STATUS_OK;
        case STOP:
            cb = readCb(in);
            stopAll();
            gTimer.at(nowMs(), cb);
            release(cb);
            return ok(out);
        case OFF:
            cb = readCb(in);
            stopAll();
            gTimer.at(nowMs(), cb);
            release(cb);
            return STATUS_OK;
        case PERFORM_HE_PARAM:
            AParcel_readInt32(in, &a);
            AParcel_readInt32(in, &b);
            AParcel_readInt32(in, &c);
            cb = readCb(in);
            {
                std::lock_guard<std::mutex> g(gLock);
                gInterval = std::max(0, a);
                gAmp = b > 0 ? clampi(b, 0, 255) : 255;
                gFreq = c;
            }
            gTimer.at(nowMs(), cb);
            release(cb);
            return ok(out);
        case SET_AMPLITUDE:
            AParcel_readInt32(in, &a);
            cb = readCb(in);
            {
                std::lock_guard<std::mutex> g(gLock);
                gAmp = clampi(a, 0, 255);
            }
            gTimer.at(nowMs(), cb);
            release(cb);
            return STATUS_OK;
        case ON: {
            AParcel_readInt32(in, &a);
            cb = readCb(in);
            int amp;
            {
                std::lock_guard<std::mutex> g(gLock);
                amp = gAmp;
            }
            gMotor.setGain(clampi(amp, 0, 255) / 255.f);
            gMotor.buzz(a);
            gTimer.at(nowMs() + a, cb);
            release(cb);
            return STATUS_OK;
        }
        case PERFORM: {
            int8_t s = 0;
            AParcel_readInt32(in, &a);
            AParcel_readByte(in, &s);
            cb = readCb(in);
            int st = static_cast<uint8_t>(s);
            Ev e;
            e.inten = st > 100 ? (st * 100 + 127) / 255 : st;
            playOnce({e}, cb);
            release(cb);
            ok(out);
            return AParcel_writeInt32(out, kTransientMs);
        }
        case PERFORM_ENVELOPE: {
            bool steep = false;
            AParcel_readInt32Array(in, &arr, allocInts);
            AParcel_readBool(in, &steep);
            cb = readCb(in);
            if (arr.size() >= 12 && arr[9] - arr[0] > 0) {
                Ev e;
                e.cont = true;
                e.inten = 100;
                e.freq = clampi(arr[2], 0, 100);
                e.dur = arr[9] - arr[0];
                for (int i = 0; i < 4; i++) {
                    e.pt.push_back(clampi(arr[3 * i] - arr[0], 0, e.dur));
                    e.pi.push_back(clampi(arr[3 * i + 1], 0, 100));
                    e.pf.push_back(clampi(arr[3 * i + 2] - e.freq, -100, 100));
                }
                playOnce({e}, cb);
            } else {
                gTimer.at(nowMs(), cb);
            }
            release(cb);
            return STATUS_OK;
        }
        case INIT:
        case SET_DYNAMIC_SCALE:
        case SET_F0:
        case SET_HAPTIC_PARAM:
        case PERFORM_RTP:
            return STATUS_OK;
        case GET_SUPPORT_FEATURES:
            ok(out);
            return AParcel_writeByteArray(out, reinterpret_cast<const int8_t*>(kFeatures),
                                          static_cast<int32_t>(strlen(kFeatures)));
        case GET_CALIBRATE_RESULTS: {
            int32_t none = 0;
            ok(out);
            return AParcel_writeInt32Array(out, &none, 0);
        }
        case GET_STATUS:
            ok(out);
            return AParcel_writeInt32(out, 0);
        case GET_TOUCH_STYLE: {
            std::lock_guard<std::mutex> g(gStyleLock);
            ok(out);
            return AParcel_writeInt32(out, gSoftStyle ? 1 : 0);
        }
        case SET_TOUCH_STYLE: {
            AParcel_readInt32(in, &a);
            std::lock_guard<std::mutex> g(gStyleLock);
            gSoftStyle = a == 1;
            return ok(out);
        }
        case WRITE_HAPTIC_AUDIO:
        case SET_POWER_ON_SWITCH:
        case LM_OFF:
        case LM_ON:
        case START_CALIBRATE:
        case SET_VMAX:
        case SET_MOTOR_OLD_TEST:
        case WRITE_GUN_TYPE:
        case WRITE_GUN_MODE:
        case WRITE_BULLET_NUM:
            return ok(out);
        case GET_INTERFACE_VERSION:
            ok(out);
            return AParcel_writeInt32(out, kOplusVersion);
        case GET_INTERFACE_HASH:
            ok(out);
            return AParcel_writeString(out, kOplusHash, static_cast<int32_t>(strlen(kOplusHash)));
        default:
            return STATUS_UNKNOWN_TRANSACTION;
    }
}

void* onCreate(void* args) { return args; }
void onDestroy(void*) {}
binder_status_t onClientTransact(AIBinder*, transaction_code_t, const AParcel*, AParcel*) {
    return STATUS_UNKNOWN_TRANSACTION;
}

AIBinder* makeServiceInstance() {
    AIBinder* vib = AIBinder_new(gVibClass, nullptr);
    AIBinder* oplus = AIBinder_new(gOplusClass, nullptr);
    gV.markVintfStability(vib);
    gV.markVintfStability(oplus);
    if (AIBinder_setExtension(vib, oplus) != STATUS_OK) {
        LOGE("AIBinder_setExtension failed");
        return nullptr;
    }
    return vib;
}

}  // namespace

int main() {
    if (!loadVndk()) return 1;
    gV.setThreadPoolMaxThreadCount(4);
    gV.startThreadPool();

    gVibClass = AIBinder_Class_define(kVibDescriptor, onCreate, onDestroy, onVibTransact);
    gOplusClass = AIBinder_Class_define(kOplusDescriptor, onCreate, onDestroy, onOplusTransact);
    gCbClass = AIBinder_Class_define(kCbDescriptor, onCreate, onDestroy, onClientTransact);
    gVibCbClass = AIBinder_Class_define(kVibCbDescriptor, onCreate, onDestroy, onClientTransact);

    gLibrary.scan();
    gLibrary.measureHeadroom();
    LOGI("touch style from settings: %s", softStyleActive() ? "soft (gentle)" : "default (crisp)");
    if (!gMotor.start()) LOGE("no motor: registering anyway so the phone boots, but nothing will vibrate");
    gLibrary.measureExcursion(gMotor.f0());

    // Register IVibrator/default (required by ColorOS 17 VibratorManagerService)
    gSelfDefault = makeServiceInstance();
    if (gSelfDefault == nullptr) return 1;
    binder_exception_t ex = gV.addService(gSelfDefault, kVibDefaultName);
    if (ex != EX_NONE) {
        LOGE("addService(%s) failed: %d", kVibDefaultName, ex);
        return 1;
    }

    // Also register IVibrator/vibratorfeature for stock Xiaomi compatibility
    gSelfFeature = makeServiceInstance();
    if (gSelfFeature != nullptr) {
        binder_exception_t ex2 = gV.addService(gSelfFeature, kVibFeatureName);
        if (ex2 != EX_NONE) LOGW("addService(%s) failed: %d (non-fatal)", kVibFeatureName, ex2);
    }

    LOGI("%s: O-Haptics vibrator HAL (IVibrator V%d + %s V%d) on psyche aw8697_haptic",
         kVibDefaultName, kVibVersion, kOplusDescriptor, kOplusVersion);
    gV.joinThreadPool();
    return 0;
}
