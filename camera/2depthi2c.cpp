#include <librealsense2/rs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

// -- CONFIGURE HERE -------------------------------------------
// D555 uses DDS (network discovery) � connect by Serial Number,
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
// and can address the slave.
static bool i2c_autodetect()
{
    for (int i = 0; I2C_BUS_CANDIDATES[i] != nullptr; ++i)
    {
        const char *bus = I2C_BUS_CANDIDATES[i];
        int fd = ::open(bus, O_RDWR);
        if (fd < 0)
        {
            std::cout << "I2C: " << bus << " not available, trying next...\n";
            continue;
        }

        if (ioctl(fd, I2C_SLAVE, I2C_SLAVE_ADDR) < 0)
        {
            std::cout << "I2C: " << bus << " cannot set slave addr 0x"
                      << std::hex << (int)I2C_SLAVE_ADDR << std::dec
                      << ", trying next...\n";
            ::close(fd);
            continue;
        }

        g_i2c_fd = fd;
        std::cout << "I2C open: " << bus
                  << " -> slave 0x" << std::hex << (int)I2C_SLAVE_ADDR
                  << std::dec << "\n";
        return true;
    }

    std::cerr << "I2C: no bus found. Is the Teensy connected?\n";
    return false;
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
// Camera context � same fields/logic as before, plus lock state
// ---------------------------------------------------------------

enum class CamState { UNKNOWN, FREE, LOCKED };

struct CamContext
{
    std::string label;     // "FRONT" / "BACK" � for printing only
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
};

static void initCamera(CamContext &cam, const std::string &serial)
{
    rs2::config cfg;

    cfg.enable_device(serial);

    cfg.enable_stream(
        RS2_STREAM_DEPTH,
        448,
        252,
        RS2_FORMAT_Z16,
        60);

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

    cam.c1 = cam.depth_w / 6;
    cam.c2 = 5 * cam.depth_w / 6;

    cam.r1a = 0;
    cam.r2a = static_cast<int>(cam.depth_h * 0.8f);

    cam.r1b = cam.r2a;
    cam.r2b = cam.depth_h;
}

// Computes dist1_mm / dist2_mm for one camera from its latest frame.
// Returns false if the frame doesn't have enough valid pixels (skip).
static bool computeDistances(CamContext &cam, float &dist1_mm, float &dist2_mm)
{
    auto frames =
        cam.pipe.wait_for_frames();

    auto depth =
        frames.get_depth_frame();

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
        (dist2_mm > 0.f && dist2_mm < THRESHOLD_MM);

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
    try
    {
        if (!i2c_autodetect())
            return 1;

        CamContext front;
        front.label    = "FRONT";
        front.lock_cmd = 101;
        front.free_cmd = 201;

        CamContext back;
        back.label    = "BACK";
        back.lock_cmd = 102;
        back.free_cmd = 202;

        initCamera(front, CAMERA_SERIAL_front);
        initCamera(back,  CAMERA_SERIAL_back);

        auto last_print  = std::chrono::steady_clock::now();
        auto last_cmd_time = std::chrono::steady_clock::now()
                              - std::chrono::milliseconds(COMMAND_COOLDOWN_MS);
        // ^ initialised in the past so the very first command isn't delayed

        while (true)
        {
            float front_dist1 = 0.f, front_dist2 = 0.f;
            float back_dist1  = 0.f, back_dist2  = 0.f;

            bool front_ok = computeDistances(front, front_dist1, front_dist2);
            bool back_ok  = computeDistances(back,  back_dist1,  back_dist2);

            if (!front_ok || !back_ok)
            {
                continue;
            }

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

            // ---- Status print (unchanged from before, just throttled) ----
            if (std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                    now - last_print)
                    .count() >= 100)
            {
                std::cout
                    << "\r[FRONT] dist1="
                    << front_dist1 / 10.f
                    << " cm   dist2="
                    << front_dist2 / 10.f
                    << " cm      "
                    << "  ||  "
                    << "[BACK] dist1="
                    << back_dist1 / 10.f
                    << " cm   dist2="
                    << back_dist2 / 10.f
                    << " cm      "
                    << std::flush;

                last_print = now;
            }
        }
    }
    catch (const rs2::error &e)
    {
        std::cerr
            << "\nRealSense Error: "
            << e.what()
            << std::endl;

        return 1;
    }
}