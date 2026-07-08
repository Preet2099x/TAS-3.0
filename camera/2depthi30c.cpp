#include <librealsense2/rs.hpp>
#include "/home/jetson/torus_service/global/GlobalState.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

// -- CONFIGURE HERE -------------------------------------------
// D555 uses DDS (network discovery)   connect by Serial Number,
// not by IP. Get this from `rs-enumerate-devices`.
static const std::string CAMERA_SERIAL_front = "419222301875";
static const std::string CAMERA_SERIAL_back  = "254822304608";

// Distance threshold in mm – same value used for both dist1/dist2 zones.
// If EITHER zone of a camera goes below this, that camera triggers LOCK.
static constexpr float THRESHOLD_MM = 500.f;   // 50 cm

// Cooldown after any I2C command is sent (ms)
static constexpr int COMMAND_COOLDOWN_MS = 500;

// I2C bus candidates – tried in order, first that opens wins
static const char *I2C_BUS_CANDIDATES[] = {
    "/dev/i2c-1",
    "/dev/i2c-0",
    nullptr   // sentinel – do not remove
};
static constexpr uint8_t I2C_SLAVE_ADDR = 0x72;

// --- Retry / resilience tuning --------------------------------
// How long to wait between camera (re)connect attempts.
static constexpr int CAMERA_RETRY_DELAY_MS = 1000;

// How often to print a "still trying" heartbeat while waiting
// for a camera to show up (so a 4-minute wait doesn't look dead).
static constexpr int CAMERA_RETRY_LOG_EVERY_MS = 5000;

// wait_for_frames() timeout per call. If a frame doesn't arrive
// in this window, rs2::error is thrown and we treat it as a miss.
static constexpr unsigned int FRAME_TIMEOUT_MS = 1500;

// If a camera misses this many frames in a row, we assume it has
// actually dropped (not just a transient hiccup) and force a full
// pipeline stop + reconnect.
static constexpr int MAX_CONSECUTIVE_FRAME_ERRORS = 5;

// How long the I2C bus autodetect keeps retrying before giving up
// entirely (set to -1 to retry forever).
static constexpr int I2C_RETRY_TIMEOUT_MS = -1;
// -------------------------------------------------------------

static float percentile5(std::vector<uint16_t> &v)
{
    if (v.empty())
        return 0.f;

    size_t idx = (v.size() - 1) / 20;
    std::nth_element(v.begin(), v.begin() + idx, v.end());

    return static_cast<float>(v[idx]);
}

static float percentile10(std::vector<uint16_t> &v)
{
    if (v.empty())
        return 0.f;

    size_t idx = (v.size() - 1) / 10;
    std::nth_element(v.begin(), v.begin() + idx, v.end());

    return static_cast<float>(v[idx]);
}

// ---------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------

static int g_i2c_fd = -1;

// Tries each I2C bus candidate in order, uses the first that opens
// and can address the slave. Keeps retrying (with a heartbeat log)
// until it succeeds, since the Teensy/USB path can come up late on
// a Jetson boot.
static bool i2c_autodetect()
{
    auto start = std::chrono::steady_clock::now();
    auto last_log = start;

    while (true)
    {
        for (int i = 0; I2C_BUS_CANDIDATES[i] != nullptr; ++i)
        {
            const char *bus = I2C_BUS_CANDIDATES[i];
            int fd = ::open(bus, O_RDWR);
            if (fd < 0)
                continue;

            if (ioctl(fd, I2C_SLAVE, I2C_SLAVE_ADDR) < 0)
            {
                ::close(fd);
                continue;
            }

            g_i2c_fd = fd;
            std::cout << "I2C open: " << bus
                      << " -> slave 0x" << std::hex << (int)I2C_SLAVE_ADDR
                      << std::dec << "\n";
            return true;
        }

        auto now = std::chrono::steady_clock::now();

        if (I2C_RETRY_TIMEOUT_MS >= 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count()
                >= I2C_RETRY_TIMEOUT_MS)
        {
            std::cerr << "I2C: no bus found after retry timeout. Is the Teensy connected?\n";
            return false;
        }

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count()
                >= CAMERA_RETRY_LOG_EVERY_MS)
        {
            std::cout << "I2C: still waiting for a bus (" << I2C_BUS_CANDIDATES[0]
                       << " / " << I2C_BUS_CANDIDATES[1] << ")...\n";
            last_log = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_RETRY_DELAY_MS));
    }
}

// Sends "NNN\n" over I2C to the Teensy (matches receiveEvent format)
static void i2c_send(int cmd)
{
    if (g_i2c_fd < 0)
        return;

    char buf[8];
    int len = snprintf(buf, sizeof(buf), "%d\n", cmd);

    if (::write(g_i2c_fd, buf, static_cast<size_t>(len)) < 0)
        std::cerr << "i2c_send: write failed – " << strerror(errno) << '\n';
}

// ---------------------------------------------------------------
// Camera context   same fields/logic as before, plus lock state
// and reconnection bookkeeping
// ---------------------------------------------------------------

enum class CamState { UNKNOWN, FREE, LOCKED };

struct CamContext
{
    std::string label;     // "FRONT" / "BACK"   for printing only
    std::string serial;    // stored so we can reconnect later
    rs2::pipeline pipe;
    float depth_scale = 0.f;

    int depth_w = 448;
    int depth_h = 252;

    int c1 = 0, c2 = 0;
    int r1a = 0, r2a = 0;
    int r1b = 0, r2b = 0;

    std::vector<uint16_t> valid1;
    std::vector<uint16_t> valid2;

    // Lock/free state machine
    CamState state = CamState::UNKNOWN;
    int lock_cmd  = 0;   // e.g. 101 for front, 102 for back
    int free_cmd  = 0;   // e.g. 201 for front, 202 for back

    // Zone B (lower 20%) threshold — can differ per camera.
    // Defaults to THRESHOLD_MM. Set lower to reduce floor sensitivity.
    float threshold_b_mm = THRESHOLD_MM;

    // Set true to ignore zone B entirely for this camera.
    bool disable_zone_b = false;

    // Reconnection bookkeeping
    bool started = false;
    int consecutive_frame_errors = 0;
};

// Fixed crop regions – independent of device state, so this is
// computed once and never needs to be redone on reconnect.
static void computeCropRegions(CamContext &cam)
{
    cam.c1 = cam.depth_w / 6;
    cam.c2 = 5 * cam.depth_w / 6;

    cam.r1a = 0;
    cam.r2a = static_cast<int>(cam.depth_h * 0.8f);

    cam.r1b = cam.r2a;
    cam.r2b = cam.depth_h;
}

// Attempts to start (or restart) the pipeline for this camera.
// Retries forever (with a heartbeat log) until it succeeds, handling:
//   - "No device connected"
//   - "No device detected. Is it plugged in?"
//   - pipeline start failures of any other kind
// This function only returns once the camera is actually streaming.
static void startCameraLoop(CamContext &cam)
{
    auto start   = std::chrono::steady_clock::now();
    auto last_log = start;
    int attempt = 0;

    while (true)
    {
        ++attempt;
        try
        {
            rs2::config cfg;
            cfg.enable_device(cam.serial);

            cfg.enable_stream(
                RS2_STREAM_DEPTH,
                cam.depth_w,
                cam.depth_h,
                RS2_FORMAT_Z16,
                30);

            rs2::pipeline_profile profile =
                cam.pipe.start(cfg);

            auto depth_sensor =
                profile.get_device().first<rs2::depth_sensor>();

            depth_sensor.set_option(
                RS2_OPTION_VISUAL_PRESET,
                4.f);

            depth_sensor.set_option(
                RS2_OPTION_ENABLE_AUTO_EXPOSURE,
                1.f);

            cam.depth_scale =
                depth_sensor.get_depth_scale();

            cam.started = true;
            cam.consecutive_frame_errors = 0;

            std::cout << "\n[" << cam.label << "] camera connected (serial "
                      << cam.serial << ") after " << attempt << " attempt(s)\n";
            return;
        }
        catch (const rs2::error &e)
        {
            // Covers "No device connected", "No device detected. Is it
            // plugged in?", and any other pipeline start failure.
            try { cam.pipe.stop(); } catch (...) { /* wasn't running, fine */ }

            auto now = std::chrono::steady_clock::now();

            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count()
                    >= CAMERA_RETRY_LOG_EVERY_MS)
            {
                auto elapsed_s =
                    std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

                std::cout << "[" << cam.label << "] still waiting for camera ("
                          << cam.serial << ") – attempt " << attempt
                          << ", " << elapsed_s << "s elapsed. Last error: "
                          << e.what() << "\n";
                last_log = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_RETRY_DELAY_MS));
        }
    }
}

static void initCamera(CamContext &cam, const std::string &serial)
{
    cam.serial = serial;
    computeCropRegions(cam);
    startCameraLoop(cam);
}

// Computes dist1_mm / dist2_mm for one camera from its latest frame.
// Returns false if the frame doesn't have enough valid pixels, if a
// frame timeout/disconnect occurred, or while a reconnect is in
// progress (i.e. any time the caller should just skip this cycle).
static bool computeDistances(CamContext &cam, float &dist1_mm, float &dist2_mm)
{
    rs2::frameset frames;

    try
    {
        frames = cam.pipe.wait_for_frames(FRAME_TIMEOUT_MS);
        cam.consecutive_frame_errors = 0;
    }
    catch (const rs2::error &e)
    {
        // Covers "Frame didn't arrive within 1500 ms" and any error
        // raised by wait_for_frames when the device has dropped.
        ++cam.consecutive_frame_errors;

        std::cout << "\n[" << cam.label << "] frame wait failed ("
                  << cam.consecutive_frame_errors << "/"
                  << MAX_CONSECUTIVE_FRAME_ERRORS << "): "
                  << e.what() << "\n";

        if (cam.consecutive_frame_errors >= MAX_CONSECUTIVE_FRAME_ERRORS)
        {
            std::cout << "[" << cam.label
                      << "] too many consecutive frame errors – "
                         "reinitializing pipeline...\n";

            try { cam.pipe.stop(); } catch (...) {}
            cam.started = false;

            // Blocks until the camera comes back – handles physical
            // disconnect/reconnect cleanly.
            startCameraLoop(cam);
        }

        return false;
    }

    auto depth = frames.get_depth_frame();

    if (!depth)
        return false;

    const uint16_t *data =
        reinterpret_cast<const uint16_t *>(
            depth.get_data());

    cam.valid1.clear();
    cam.valid2.clear();

    for (int y = cam.r1a; y < cam.r2a; ++y)
    {
        for (int x = cam.c1; x < cam.c2; ++x)
        {
            uint16_t d =
                data[y * cam.depth_w + x];

            if (d > 0)
                cam.valid1.push_back(d);
        }
    }

    for (int y = cam.r1b; y < cam.r2b; ++y)
    {
        for (int x = cam.c1; x < cam.c2; ++x)
        {
            uint16_t d =
                data[y * cam.depth_w + x];

            if (d > 0)
                cam.valid2.push_back(d);
        }
    }

    if (cam.valid1.size() < 100 ||
        cam.valid2.size() < 100)
    {
        return false;
    }

    dist1_mm =
        percentile5(cam.valid1) *
        cam.depth_scale *
        1000.f;

    dist2_mm =
        percentile10(cam.valid2) *
        cam.depth_scale *
        1000.f;

    return true;
}

// Checks this camera's two distances against THRESHOLD_MM and updates
// its state. Returns the command to send (0 = no change, send nothing).
static int updateState(CamContext &cam, float dist1_mm, float dist2_mm)
{
    bool triggered =
        (dist1_mm > 0.f && dist1_mm < THRESHOLD_MM) ||
        (!cam.disable_zone_b && dist2_mm > 0.f && dist2_mm < cam.threshold_b_mm);

    CamState next = cam.state;

    if (triggered)
        next = CamState::LOCKED;
    else
        next = CamState::FREE;

    if (next != cam.state)
    {
        cam.state = next;
        return (next == CamState::LOCKED) ? cam.lock_cmd : cam.free_cmd;
    }

    return 0;   // no state change, nothing to send
}

int main()
{
    if (!i2c_autodetect())
        return 1;

    CamContext front;
    front.label    = "FRONT";
    front.lock_cmd = 101;
    front.free_cmd = 201;

    CamContext back;
    back.label          = "BACK";
    back.lock_cmd       = 102;
    back.free_cmd       = 202;
    back.threshold_b_mm = 300.f;   // 30 cm — less sensitive floor zone
    back.disable_zone_b = true;    // floor zone fully disabled for back camera

    // These block until each camera is actually streaming – no matter
    // how long that takes (device not found, plugged in late, etc).
    initCamera(front, CAMERA_SERIAL_front);
    initCamera(back,  CAMERA_SERIAL_back);

    // Both cameras ready — announce startup cleanly.
    // Reset state to UNKNOWN so the first FREE/LOCK log after this is
    // a real state change, not the startup flush.
    front.state = CamState::UNKNOWN;
    back.state  = CamState::UNKNOWN;

    std::cout << "\nDEPTH STARTED\n\n" << std::flush;

    auto last_print  = std::chrono::steady_clock::now();
    auto last_cmd_time = std::chrono::steady_clock::now()
                          - std::chrono::milliseconds(COMMAND_COOLDOWN_MS);
    // ^ initialised in the past so the very first command isn't delayed

    // Tracks whether we have already sent 201+202 after depthToggle went off.
    // Reset to false when depthToggle comes back on, so next pause sends again.
    bool depth_paused_free_sent = false;

    while (true)
    {
        // Outer safety net: nothing below this point should be able to
        // kill the process. Any stray rs2::error is logged and the loop
        // just continues on the next iteration instead of exiting.
        try
        {
            // Poll GlobalState — if depthToggle is false, depth is
            // disabled by the system. Send free commands once to release
            // any locked state, then pause until re-enabled.
            if (!GlobalState::instance().depthToggle())
            {
                if (!depth_paused_free_sent)
                {
                    i2c_send(201);
                    std::this_thread::sleep_for(std::chrono::milliseconds(COMMAND_COOLDOWN_MS));
                    i2c_send(202);

                    front.state = CamState::FREE;
                    back.state  = CamState::FREE;
                    depth_paused_free_sent = true;
                    last_cmd_time = std::chrono::steady_clock::now();

                    std::cout << "\nDEPTH PAUSED — sent 201 + 202 (force FREE)\n" << std::flush;
                }
                else
                {
                    std::cout << "DEPTH PAUSED\n" << std::flush;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
}

            // depthToggle is on — reset flag so next pause sends free again
            depth_paused_free_sent = false;

            float front_dist1 = 0.f, front_dist2 = 0.f;
            float back_dist1  = 0.f, back_dist2  = 0.f;

            bool front_ok = computeDistances(front, front_dist1, front_dist2);
            bool back_ok  = computeDistances(back,  back_dist1,  back_dist2);

            if (!front_ok || !back_ok)
            {
                continue;
            }

            // New, valid depth data is available for both cameras this
            // cycle — announce it the same way DEPTH STARTED is
            // announced at boot.
            std::cout << "DEPTH ONLINE\n";

            auto now = std::chrono::steady_clock::now();

            // ---- Lock/free state machine, respecting the 500ms cooldown ----
            auto since_last_cmd =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_cmd_time)
                    .count();

            if (since_last_cmd >= COMMAND_COOLDOWN_MS)
            {
                int front_cmd = updateState(front, front_dist1, front_dist2);
                if (front_cmd != 0)
                {
                    i2c_send(front_cmd);
                    last_cmd_time = now;

                    std::cout << "\n[FRONT] sent " << front_cmd
                              << (front_cmd == front.lock_cmd ? "  (LOCK)" : "  (FREE)")
                              << "\n";
                }
                else
                {
                    int back_cmd = updateState(back, back_dist1, back_dist2);
                    if (back_cmd != 0)
                    {
                        i2c_send(back_cmd);
                        last_cmd_time = now;

                        std::cout << "\n[BACK] sent " << back_cmd
                                  << (back_cmd == back.lock_cmd ? "  (LOCK)" : "  (FREE)")
                                  << "\n";
                    }
                }
            }

            // ---- Status print ----
            if (std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                    now - last_print)
                    .count() >= 1000)   // print once per second
            {
                std::cout
                    << "[FRONT] dist1="
                    << front_dist1 / 10.f
                    << " cm   dist2="
                    << front_dist2 / 10.f
                    << " cm"
                    << "  ||  "
                    << "[BACK] dist1="
                    << back_dist1 / 10.f
                    << " cm   dist2="
                    << back_dist2 / 10.f
                    << " cm\n";

                last_print = now;
            }
        }
        catch (const rs2::error &e)
        {
            std::cerr
                << "\nRealSense Error (main loop, continuing): "
                << e.what()
                << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_RETRY_DELAY_MS));
        }
    }
}