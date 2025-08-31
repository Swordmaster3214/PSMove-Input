#include "Types.h"
#include "UinputDevice.h"
#include "PSMovePoller.h"
#include "FFHandler.h"
#include "Mapper.h"
#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>

static std::atomic<bool> g_quit{false};
static void sigint_handler(int){ g_quit = true; }

int main() {
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    ThreadQueue<Sample> q;
    UinputDevice uidev;
    if (!uidev.create("Virtual DualShock4 (Move/Nav bridge)")) {
        std::cerr<<"Failed to create uinput device\n"; return 1;
    }

    PSMovePoller poller(q);
    if (!poller.start()) {
        std::cerr<<"PSMovePoller failed to start (no controllers?)\n";
        // continue anyway — you may connect controllers later and extend to reinit
    }

    // FF handler forwards rumble to poller
    FFHandler ff(uidev, [&](uint8_t rumble){ poller.set_rumble_all(rumble); });
    ff.start();

    Mapper mapper(uidev);

    // Consumer: process samples from PSMovePoller and feed mapper/uinput
    std::thread consumer([&](){
        Sample s;
        while (!g_quit.load() && q.wait_pop(s)) {
            mapper.map_and_feed(s);
        }
    });

    std::cerr<<"Bridge running. Ctrl-C to quit.\n";
    while (!g_quit.load()) std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cerr<<"Shutting down...\n";
    q.close();
    consumer.join();
    ff.stop();
    poller.stop();
    uidev.destroy();
    return 0;
}
