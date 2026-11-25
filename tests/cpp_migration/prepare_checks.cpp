// This test only uses synthetic interfaces/mode replies/packets and local
// printf/sleep child processes. It never constructs a DDS participant.
#define G1_PREPARE_NO_MAIN
#include "../../deploy/robots/g1/prepare_main.cpp"

namespace {
void expect(bool ok, const char* description) { if (!ok) throw std::runtime_error(description); }
template<class Function> void rejected(Function function) {
    try { function(); } catch (const std::exception&) { return; }
    throw std::runtime_error("invalid preparation input was accepted");
}
}
int main() {
    using namespace g1prepare;
    try {
        const auto address = parseAddress("192.168.123.222/24");
        expect(address.contains(ipv4("192.168.123.161")), "subnet membership failed");
        expect(!address.contains(ipv4("192.168.124.161")), "foreign subnet accepted");
        rejected([] { parseAddress("192.168.1.1/33"); });
        rejected([] { parseAddress("192.168.1.1"); });
        rejected([] { ipv4("192.168.1.999"); });
        Interface ethernet{"enp-test", true, true, false, false, {address}};
        Interface other{"other", true, true, false, false, {parseAddress("10.2.3.4/24")}};
        Interface wifi{"wifi", false, true, false, true, {address}};
        Interface loopback{"lo", false, true, true, false, {parseAddress("127.0.0.1/8")}};
        expect(chooseInterface({ethernet, other, wifi}, "", address).name == "enp-test", "unique same-subnet choice failed");
        rejected([&] { chooseInterface({ethernet, wifi}, "wifi", address); });
        rejected([&] { chooseInterface({loopback}, "lo", address); });
        rejected([&] { chooseInterface({ethernet}, "missing", address); });
        auto ambiguous = other; ambiguous.addresses = {address};
        rejected([&] { chooseInterface({ethernet, ambiguous}, "", address); });
        auto unplugged = ethernet; unplugged.connected = false;
        expect(chooseInterface({unplugged, wifi}, "", address).name == "enp-test", "single unplugged Ethernet was not selected");
        auto plan = networkCommands(ethernet, address, "192.168.123.161", false);
        expect(plan.size() == 2 && plan[0][0] == "sudo" && plan[1][3] == "replace", "existing-address network plan mismatch");
        auto empty = ethernet; empty.addresses.clear();
        plan = networkCommands(empty, address, "192.168.123.161", true);
        expect(plan.size() == 3 && plan[1][1] == "address" && plan[1][2] == "add", "missing-address plan mismatch");
        expect(plan[2][3] == "192.168.123.161/32", "host route did not preserve default routing");
        for (const auto& command : plan) for (const auto& argument : command)
            expect(argument != "flush" && argument != "default" && argument != "delete", "destructive network plan");
        auto conflicting = ethernet; conflicting.addresses = {parseAddress("192.168.123.222/16")};
        rejected([&] { networkCommands(conflicting, address, "192.168.123.161"); });

        int releases = 0, checks = 0;
        auto wait = [](double) {};
        auto mode = ensureMode([] { return std::make_pair(0, std::string()); }, [&] { ++releases; return 0; }, true, 3, wait);
        expect(mode.inactive && releases == 0, "inactive mode was released");
        mode = ensureMode([] { return std::make_pair(0, std::string("ai")); }, [&] { ++releases; return 0; }, false, 3, wait);
        expect(!mode.inactive && releases == 0, "check-only released AI mode");
        mode = ensureMode([] { return std::make_pair(0, std::string("custom")); }, [&] { ++releases; return 0; }, true, 3, wait);
        expect(!mode.inactive && releases == 0, "unrecognized active mode was released");
        mode = ensureMode([&] { return std::make_pair(0, checks++ ? std::string() : std::string("ai")); },
                          [&] { ++releases; return -1; }, true, 3, wait);
        expect(mode.inactive && releases == 1 && checks == 2 && mode.release_codes[0] == -1,
               "release timeout was not followed by a confirming query");
        releases = 0;
        rejected([&] { ensureMode([] { return std::make_pair(-2, std::string()); }, [&] { ++releases; return 0; }, true, 3, wait); });
        expect(releases == 0, "unknown controller query result permitted release");
        checks = 0;
        mode = ensureMode([&] { return std::make_pair(0, checks++ ? std::string("custom") : std::string("ai")); },
                          [&] { ++releases; return 0; }, true, 3, wait);
        expect(!mode.inactive && releases == 1, "release continued after a different mode took ownership");

        LowState packet{}; packet.mode_machine() = 5; packet.tick() = 1; packet.crc() = crc(packet);
        ProbeStats stats; stats.accept(packet); packet.tick() = 2; packet.crc() = crc(packet); stats.accept(packet);
        ModeResult quiet; quiet.inactive = true;
        expect(readyResult(quiet, stats, .01)["ready"].as<bool>(), "valid readiness rejected");
        expect(!readyResult(quiet, stats, .11)["ready"].as<bool>(), "stale readiness accepted");
        stats.commands = 1;
        expect(!readyResult(quiet, stats, .01)["ready"].as<bool>(), "competing writer accepted");
        stats.commands = 0; packet.crc() ^= 1; stats.accept(packet);
        expect(!readyResult(quiet, stats, .01)["ready"].as<bool>(), "bad CRC readiness accepted");

        const auto literal = run({"printf", "%s", "$(never_execute); spaces and 'quotes'"});
        expect(literal.code == 0 && literal.output == "$(never_execute); spaces and 'quotes'", "argv was interpreted by a shell");
        expect(run({"sleep", "1"}, .02).timed_out, "subprocess timeout failed");
        ProcessResult worker; worker.code = 0;
        worker.output = "SDK diagnostic\nG1_PREP_RESULT={\"ready\":true,\"mode\":\"ai\"}\n";
        expect(parseWorkerResult(worker)["ready"].as<bool>(), "worker JSON result parsing failed");
        auto data = YAML::Load(json(interfaceNode(ethernet)));
        expect(data["ifname"].as<std::string>() == ethernet.name, "JSON report roundtrip failed");
        std::cout << "PASS: preparation interface selection, network argv, limited ReleaseMode, readiness, JSON, subprocess timeout\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
