// Native equivalent of supermimic_reference/scripts/deploy/prepare_g1.py.
// This executable never constructs a LowCmd publisher or starts a policy.
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

extern char** environ;

namespace g1prepare {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
constexpr const char* kResultPrefix = "G1_PREP_RESULT=";

void pause(double duration) { std::this_thread::sleep_for(std::chrono::duration<double>(duration)); }
double elapsed(Clock::time_point since) { return std::chrono::duration<double>(Clock::now()-since).count(); }

YAML::Node stringNode(const std::string& text) {
    YAML::Node node(text); node.SetTag("tag:yaml.org,2002:str"); return node;
}
std::string quote(const std::string& text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : text) {
        switch (c) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
            else out << static_cast<char>(c);
        }
    }
    return out.str()+'"';
}
std::string json(const YAML::Node& node) {
    if (!node || node.IsNull()) return "null";
    if (node.IsSequence()) {
        std::string text = "[";
        for (std::size_t i = 0; i < node.size(); ++i) { if (i) text += ','; text += json(node[i]); }
        return text+']';
    }
    if (node.IsMap()) {
        std::string text = "{"; bool first = true;
        for (const auto& item : node) {
            if (!first) text += ',';
            first = false; text += quote(item.first.as<std::string>())+':'+json(item.second);
        }
        return text+'}';
    }
    const auto scalar = node.Scalar();
    if (node.Tag() == "tag:yaml.org,2002:str" || node.Tag() == "!") return quote(scalar);
    if (scalar == "true" || scalar == "false" || scalar == "null") return scalar;
    char* end = nullptr;
    const double value = std::strtod(scalar.c_str(), &end);
    if (!scalar.empty() && end == scalar.c_str()+scalar.size() && std::isfinite(value)) return scalar;
    return quote(scalar);
}
YAML::Node argvNode(const std::vector<std::string>& argv) {
    YAML::Node result(YAML::NodeType::Sequence);
    for (const auto& argument : argv) result.push_back(stringNode(argument));
    return result;
}

struct ProcessResult { int code = -1; std::string output, error; bool timed_out = false; };
ProcessResult run(const std::vector<std::string>& arguments, double timeout = 20.) {
    if (arguments.empty()) throw std::invalid_argument("empty process argv");
    int output_pipe[2], error_pipe[2];
    if (pipe2(output_pipe, O_CLOEXEC) != 0) throw std::runtime_error("cannot create stdout pipe");
    if (pipe2(error_pipe, O_CLOEXEC) != 0) {
        close(output_pipe[0]); close(output_pipe[1]);
        throw std::runtime_error("cannot create stderr pipe");
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, output_pipe[1], STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, error_pipe[1], STDERR_FILENO);
    for (int descriptor : {output_pipe[0], output_pipe[1], error_pipe[0], error_pipe[1]})
        posix_spawn_file_actions_addclose(&actions, descriptor);
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attributes, 0);
    std::vector<char*> argv;
    for (const auto& value : arguments) argv.push_back(const_cast<char*>(value.c_str()));
    argv.push_back(nullptr);
    pid_t process = -1;
    const int started = posix_spawnp(&process, argv[0], &actions, &attributes, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions); posix_spawnattr_destroy(&attributes);
    close(output_pipe[1]); close(error_pipe[1]);
    if (started != 0) {
        close(output_pipe[0]); close(error_pipe[0]);
        throw std::runtime_error(arguments[0]+": "+std::strerror(started));
    }
    fcntl(output_pipe[0], F_SETFL, O_NONBLOCK); fcntl(error_pipe[0], F_SETFL, O_NONBLOCK);
    ProcessResult result;
    auto drain = [](int descriptor, std::string& text) {
        char buffer[4096]; ssize_t count;
        while ((count = ::read(descriptor, buffer, sizeof(buffer))) > 0) text.append(buffer, static_cast<std::size_t>(count));
    };
    const auto begin = Clock::now();
    int status = 0;
    while (true) {
        drain(output_pipe[0], result.output); drain(error_pipe[0], result.error);
        const auto waited = waitpid(process, &status, WNOHANG);
        if (waited == process) break;
        if (waited < 0 && errno != EINTR) break;
        if (elapsed(begin) > timeout) {
            result.timed_out = true;
            kill(-process, SIGTERM);
            pause(.1);
            if (waitpid(process, &status, WNOHANG) != process) {
                kill(-process, SIGKILL);
                while (waitpid(process, &status, 0) < 0 && errno == EINTR) {}
            }
            break;
        }
        pollfd descriptors[2]{{output_pipe[0], POLLIN, 0}, {error_pipe[0], POLLIN, 0}};
        poll(descriptors, 2, 30);
    }
    drain(output_pipe[0], result.output); drain(error_pipe[0], result.error);
    close(output_pipe[0]); close(error_pipe[0]);
    result.code = result.timed_out ? 124 : WIFEXITED(status) ? WEXITSTATUS(status) : 128+WTERMSIG(status);
    return result;
}

std::uint32_t ipv4(const std::string& text) {
    in_addr address{};
    if (inet_pton(AF_INET, text.c_str(), &address) != 1)
        throw std::invalid_argument("invalid IPv4 address: "+text);
    return ntohl(address.s_addr);
}
struct Address {
    std::string ip;
    unsigned prefix = 0;
    std::uint32_t value = 0, mask = 0;
    std::string cidr() const { return ip+'/'+std::to_string(prefix); }
    bool contains(std::uint32_t other) const { return (other & mask) == (value & mask); }
};
Address parseAddress(const std::string& text) {
    const auto separator = text.find('/');
    if (separator == std::string::npos) throw std::invalid_argument("--address requires IPv4/prefix");
    Address result;
    result.ip = text.substr(0, separator); result.value = ipv4(result.ip);
    const auto prefix = text.substr(separator+1);
    if (prefix.empty() || prefix.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("invalid IPv4 prefix");
    const auto bits = std::stoul(prefix);
    if (bits > 32) throw std::invalid_argument("invalid IPv4 prefix");
    result.prefix = bits; result.mask = bits == 0 ? 0 : 0xffffffffu << (32-bits);
    return result;
}
struct Interface {
    std::string name;
    bool wired = false, connected = false, loopback = false, wireless = false;
    std::vector<Address> addresses;
};
bool carrier(const std::string& name) {
    std::ifstream input(fs::path("/sys/class/net")/name/"carrier");
    int value = 0; return bool(input >> value) && value == 1;
}
std::vector<Interface> interfaces() {
    ifaddrs* raw = nullptr;
    if (getifaddrs(&raw) != 0) throw std::runtime_error("getifaddrs failed");
    const std::unique_ptr<ifaddrs, decltype(&freeifaddrs)> addresses(raw, freeifaddrs);
    std::map<std::string, Interface> grouped;
    for (auto* entry = raw; entry; entry = entry->ifa_next) {
        auto& row = grouped[entry->ifa_name]; row.name = entry->ifa_name;
        row.loopback = (entry->ifa_flags & IFF_LOOPBACK) != 0;
        const auto node = fs::path("/sys/class/net")/row.name;
        row.wireless = fs::exists(node/"wireless");
        row.wired = fs::exists(node/"device") && !row.wireless && !row.loopback;
        row.connected = carrier(row.name);
        if (!entry->ifa_addr || entry->ifa_addr->sa_family != AF_INET || !entry->ifa_netmask) continue;
        const auto* address = reinterpret_cast<const sockaddr_in*>(entry->ifa_addr);
        const auto* netmask = reinterpret_cast<const sockaddr_in*>(entry->ifa_netmask);
        char ip[INET_ADDRSTRLEN]{};
        if (!inet_ntop(AF_INET, &address->sin_addr, ip, sizeof(ip))) continue;
        const auto mask = ntohl(netmask->sin_addr.s_addr);
        unsigned bits = 0;
        for (auto bit = mask; bit; bit <<= 1) ++bits;
        row.addresses.push_back(parseAddress(std::string(ip)+'/'+std::to_string(bits)));
    }
    std::vector<Interface> result;
    for (const auto& item : grouped) result.push_back(item.second);
    return result;
}
Interface chooseInterface(const std::vector<Interface>& rows, const std::string& requested, const Address& network) {
    if (!requested.empty()) {
        for (const auto& row : rows) if (row.name == requested) {
            if (row.loopback || row.name == "lo") throw std::runtime_error("loopback is not a robot Ethernet interface");
            if (row.wireless) throw std::runtime_error("a wired robot interface is required; Wi-Fi cannot be prepared");
            return row;
        }
        throw std::runtime_error("interface "+requested+" was not found");
    }
    std::vector<Interface> wired, connected, same;
    for (const auto& row : rows) {
        if (!row.wired) continue;
        wired.push_back(row);
        if (!row.connected) continue;
        connected.push_back(row);
        if (std::any_of(row.addresses.begin(), row.addresses.end(), [&](const Address& a) { return network.contains(a.value); }))
            same.push_back(row);
    }
    if (same.size() == 1) return same.front();
    if (connected.size() == 1) return connected.front();
    if (connected.empty() && wired.size() == 1) return wired.front();
    throw std::runtime_error("cannot select one wired robot interface; pass --iface NAME (Wi-Fi is never selected)");
}
std::vector<std::vector<std::string>> networkCommands(const Interface& row, const Address& address,
                                                     const std::string& robot_ip, bool root = geteuid() == 0) {
    bool present = false;
    for (const auto& item : row.addresses) if (item.value == address.value) {
        if (item.prefix != address.prefix)
            throw std::runtime_error("requested IP already exists with another prefix; other addresses are preserved");
        present = true;
    }
    std::vector<std::vector<std::string>> commands{{"ip", "link", "set", "dev", row.name, "up"}};
    if (!present) commands.push_back({"ip", "address", "add", address.cidr(), "dev", row.name});
    commands.push_back({"ip", "route", "replace", robot_ip+"/32", "dev", row.name, "src", address.ip});
    if (!root) for (auto& command : commands) command.insert(command.begin(), "sudo");
    return commands;
}
YAML::Node interfaceNode(const Interface& row) {
    YAML::Node result;
    result["ifname"] = stringNode(row.name); result["wired"] = row.wired; result["connected"] = row.connected;
    result["addr_info"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& address : row.addresses) {
        YAML::Node entry; entry["local"] = stringNode(address.ip); entry["prefixlen"] = address.prefix;
        result["addr_info"].push_back(entry);
    }
    return result;
}

struct ModeResult { std::string before, after; std::vector<int> release_codes; bool inactive = false; };
using CheckMode = std::function<std::pair<int, std::string>()>;
std::string validMode(const CheckMode& check) {
    const auto reply = check();
    if (reply.first != 0) throw std::runtime_error("MotionSwitcher has not confirmed its mode: code="+std::to_string(reply.first));
    return reply.second;
}
ModeResult ensureMode(const CheckMode& check, const std::function<int()>& release,
                      bool release_allowed, int attempts, const std::function<void(double)>& wait = pause) {
    ModeResult result;
    bool queried = false;
    std::string last_error;
    for (int attempt = 0; attempt < attempts; ++attempt) {
        try { result.after = validMode(check); queried = true; break; }
        catch (const std::exception& error) { last_error = error.what(); wait(.5); }
    }
    if (!queried) throw std::runtime_error(last_error);
    result.before = result.after;
    for (int attempt = 0; release_allowed && result.after == "ai" && attempt < attempts; ++attempt) {
        result.release_codes.push_back(release());
        wait(.7);
        // A timeout from ReleaseMode is not success; always confirm via GET.
        result.after = validMode(check);
    }
    result.inactive = result.after.empty();
    return result;
}
YAML::Node modeNode(const ModeResult& mode) {
    YAML::Node result; result["before"] = stringNode(mode.before); result["after"] = stringNode(mode.after);
    result["inactive"] = mode.inactive; result["events"] = YAML::Node(YAML::NodeType::Sequence);
    for (auto code : mode.release_codes) { YAML::Node event; event["release_code"] = code; result["events"].push_back(event); }
    return result;
}
template<class Message> std::uint32_t crc(const Message& message) {
    std::uint32_t result = 0xffffffff;
    const auto* bytes = reinterpret_cast<const unsigned char*>(&message);
    for (std::size_t index = 0; index < sizeof(Message)/4-1; ++index) {
        std::uint32_t word; std::memcpy(&word, bytes+4*index, 4);
        for (std::uint32_t mask = 0x80000000; mask; mask >>= 1) {
            const bool high = (result & 0x80000000) != 0; result <<= 1;
            if (high) result ^= 0x04c11db7;
            if (word & mask) result ^= 0x04c11db7;
        }
    }
    return result;
}
struct ProbeStats {
    std::uint64_t samples = 0, tick_changes = 0, crc_bad = 0, commands = 0;
    std::uint32_t tick = 0;
    int mode_machine = -1;
    Clock::time_point last{};
    void accept(const LowState& state) {
        if (samples && tick != state.tick()) ++tick_changes;
        ++samples; crc_bad += state.crc() != crc(state);
        tick = state.tick(); mode_machine = state.mode_machine(); last = Clock::now();
    }
};
YAML::Node readyResult(const ModeResult& mode, const ProbeStats& state, double age) {
    YAML::Node reasons(YAML::NodeType::Sequence);
    if (!mode.inactive) reasons.push_back(stringNode("motion controller is still active"));
    if (state.samples < 2 || !state.tick_changes) reasons.push_back(stringNode("no advancing lowstate"));
    if (!std::isfinite(age) || age < 0 || age > .1) reasons.push_back(stringNode("lowstate is stale"));
    if (state.crc_bad) reasons.push_back(stringNode("lowstate CRC errors"));
    if (state.mode_machine != 5 && state.mode_machine != 6) reasons.push_back(stringNode("unsupported G1 mode_machine"));
    if (state.commands) reasons.push_back(stringNode("rt/lowcmd still has another publisher"));
    YAML::Node result; result["ready"] = reasons.size() == 0; result["reasons"] = reasons;
    result["mode"] = modeNode(mode);
    result["lowstate"]["samples"] = state.samples; result["lowstate"]["tick_changes"] = state.tick_changes;
    result["lowstate"]["crc_bad"] = state.crc_bad; result["lowstate"]["mode_machine"] = state.mode_machine;
    result["lowstate"]["age_s"] = age;
    result["other_lowcmd_samples"] = state.commands;
    result["scope"] = stringNode("network and controller ownership only; not policy or physical deployment qualification");
    return result;
}
struct Options {
    std::string iface, address = "192.168.123.222/24", robot_ip = "192.168.123.161";
    int domain = 0, retries = 3;
    double timeout = 5., quiet_seconds = 1.;
    bool check_only = false, dry_run = false, probe_worker = false;
    fs::path report_dir = "prepare_logs";
};
YAML::Node probe(const Options& options) {
    unitree::robot::ChannelFactory::Instance()->Init(options.domain, options.iface);
    struct Shared { std::mutex mutex; ProbeStats stats; };
    const auto shared = std::make_shared<Shared>();
    unitree::robot::ChannelSubscriber<LowState> subscriber("rt/lowstate");
    unitree::robot::ChannelSubscriber<LowCmd> watcher("rt/lowcmd");
    subscriber.InitChannel([shared](const void* message) {
        std::lock_guard<std::mutex> lock(shared->mutex);
        shared->stats.accept(*static_cast<const LowState*>(message));
    }, 0);
    watcher.InitChannel([shared](const void*) { std::lock_guard<std::mutex> lock(shared->mutex); ++shared->stats.commands; }, 0);
    const auto started = Clock::now();
    bool enough = false;
    while (elapsed(started) < options.timeout) {
        { std::lock_guard<std::mutex> lock(shared->mutex); enough = shared->stats.samples >= 2 && shared->stats.tick_changes > 0; }
        if (enough) break;
        pause(.05);
    }
    if (!enough) throw std::runtime_error("no advancing rt/lowstate: verify cable, power, interface, domain and multicast rules; AI mode was not changed");
    {
        std::lock_guard<std::mutex> lock(shared->mutex);
        if ((shared->stats.mode_machine != 5 && shared->stats.mode_machine != 6) || shared->stats.crc_bad)
            throw std::runtime_error("G1 model/CRC validation failed; AI mode was not changed");
    }
    unitree::robot::b2::MotionSwitcherClient client;
    client.SetTimeout(2.f); client.Init();
    const auto check = [&] { std::string form, name; const int code = client.CheckMode(form, name); return std::make_pair(code, name); };
    auto mode = ensureMode(check, [&] { return client.ReleaseMode(); }, !options.check_only, options.retries);
    { std::lock_guard<std::mutex> lock(shared->mutex); shared->stats.commands = 0; }
    pause(options.quiet_seconds);
    mode.after = validMode(check); mode.inactive = mode.after.empty();
    ProbeStats state;
    { std::lock_guard<std::mutex> lock(shared->mutex); state = shared->stats; }
    const auto result = readyResult(mode, state, elapsed(state.last));
    subscriber.CloseChannel(); watcher.CloseChannel();
    return result;
}
std::string executablePath() {
    std::array<char, 8192> buffer{};
    const auto size = readlink("/proc/self/exe", buffer.data(), buffer.size()-1);
    if (size <= 0) throw std::runtime_error("cannot resolve the preparation executable");
    return std::string(buffer.data(), static_cast<std::size_t>(size));
}
YAML::Node parseWorkerResult(const ProcessResult& process) {
    if (process.timed_out) { YAML::Node result; result["ready"] = false; result["error"] = stringNode("DDS probe exceeded its bounded timeout"); return result; }
    const auto start = process.output.rfind(kResultPrefix);
    if (start != std::string::npos) {
        const auto begin = start+std::strlen(kResultPrefix);
        return YAML::Load(process.output.substr(begin, process.output.find('\n', begin)-begin));
    }
    YAML::Node result; result["ready"] = false;
    result["error"] = stringNode("probe exited "+std::to_string(process.code)+": "+process.error+process.output);
    return result;
}
std::string timestamp() {
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local{}; localtime_r(&now, &local);
    std::ostringstream out; out << std::put_time(&local, "%Y%m%d_%H%M%S"); return out.str();
}

int mainImpl(int argc, char** argv) {
    namespace po = boost::program_options;
    Options options;
    std::string report_dir = options.report_dir.string();
    po::options_description visible("Native G1 network/control preparation (no motor-command publisher)");
    visible.add_options()
        ("help,h", "show help")("iface", po::value<std::string>(&options.iface), "wired robot NIC; auto-select only when unambiguous")
        ("address", po::value<std::string>(&options.address)->default_value(options.address), "PC IPv4/prefix")
        ("robot-ip", po::value<std::string>(&options.robot_ip)->default_value(options.robot_ip), "ping/host-route target; DDS lowstate is authoritative")
        ("domain", po::value<int>(&options.domain)->default_value(0), "real robot DDS domain (must be 0)")
        ("check-only", po::bool_switch(&options.check_only), "no network changes or ReleaseMode; read-only DDS/RPC checks")
        ("dry-run", po::bool_switch(&options.dry_run), "print network plan; no network changes, DDS, RPC, or ping")
        ("retries", po::value<int>(&options.retries)->default_value(3), "1..5 fresh-process probe attempts")
        ("timeout", po::value<double>(&options.timeout)->default_value(5.), "1..30 s lowstate wait per probe")
        ("quiet-seconds", po::value<double>(&options.quiet_seconds)->default_value(1.), "1..5 s LowCmd silence check")
        ("report-dir", po::value<std::string>(&report_dir)->default_value(report_dir), "JSON reports directory");
    po::options_description all; all.add(visible).add_options()("probe-worker", po::bool_switch(&options.probe_worker), "internal fresh-process probe");
    po::variables_map args;
    try {
        po::store(po::parse_command_line(argc, argv, all), args); po::notify(args);
        if (args.count("help")) { std::cout << visible << '\n'; return 0; }
        if (options.domain != 0) throw std::invalid_argument("preparation is for real G1 domain 0 only");
        if (options.retries < 1 || options.retries > 5 || !std::isfinite(options.timeout) || options.timeout < 1 || options.timeout > 30 ||
            !std::isfinite(options.quiet_seconds) || options.quiet_seconds < 1 || options.quiet_seconds > 5)
            throw std::invalid_argument("retries must be 1..5, timeout 1..30 s, quiet-seconds 1..5 s");
    } catch (const std::exception& error) { std::cerr << "NOT READY: " << error.what() << '\n'; return 2; }
    options.report_dir = report_dir;
    // Dry-run takes precedence even if the internal worker flag was supplied.
    if (options.probe_worker && !options.dry_run) {
        YAML::Node result;
        try {
            if (options.iface.empty() || options.iface == "lo") throw std::invalid_argument("worker requires an explicit wired interface");
            chooseInterface(interfaces(), options.iface, parseAddress(options.address));
            result = probe(options);
        } catch (const std::exception& error) { result["ready"] = false; result["error"] = stringNode(error.what()); }
        std::cout << kResultPrefix << json(result) << '\n';
        return result["ready"].as<bool>() ? 0 : 1;
    }
    YAML::Node report;
    report["started"] = stringNode(timestamp()); report["os"] = stringNode("Linux");
    report["runtime"] = stringNode("native C++"); report["actions"] = YAML::Node(YAML::NodeType::Sequence);
    report["check_only"] = options.check_only;
    const auto path = options.report_dir/("prepare_"+timestamp()+"_"+std::to_string(getpid())+".json");
    int exit_code = 2;
    try {
        fs::create_directories(options.report_dir);
        const auto address = parseAddress(options.address);
        const auto robot_ip = ipv4(options.robot_ip);
        if (!address.contains(robot_ip) || address.value == robot_ip)
            throw std::invalid_argument("robot and PC need distinct IPv4 addresses on the same subnet");
        const auto row = chooseInterface(interfaces(), options.iface, address);
        options.iface = row.name;
        report["iface"] = stringNode(row.name); report["address"] = stringNode(address.cidr());
        report["robot_ip"] = stringNode(options.robot_ip); report["network_before"] = interfaceNode(row);
        const auto commands = networkCommands(row, address, options.robot_ip);
        if (options.dry_run) {
            report["dry_run"] = true; report["ready"] = false;
            report["planned_commands"] = YAML::Node(YAML::NodeType::Sequence);
            if (!options.check_only) for (const auto& command : commands) report["planned_commands"].push_back(argvNode(command));
            std::cout << json(report) << '\n'; exit_code = 0;
        } else {
            if (!options.check_only) {
                std::cout << "Robot must be supported by the gantry: AI mode will be released automatically.\n" << std::flush;
                for (const auto& command : commands) {
                    const auto outcome = run(command);
                    if (outcome.code != 0) throw std::runtime_error(command[0]+" failed: "+outcome.error+outcome.output);
                    report["actions"].push_back(argvNode(command));
                }
            }
            bool linked = false;
            for (int attempt = 0; attempt < 10; ++attempt) { if ((linked = carrier(row.name))) break; pause(.3); }
            if (!linked) throw std::runtime_error(row.name+": no Ethernet carrier; connect cable and power on robot; AI mode was not changed");
            try { report["ping_ok"] = run({"ping", "-c", "1", options.robot_ip}, 3.).code == 0; }
            catch (const std::exception&) { report["ping_ok"] = false; }
            report["probes"] = YAML::Node(YAML::NodeType::Sequence);
            YAML::Node result;
            for (int attempt = 0; attempt < options.retries; ++attempt) {
                std::vector<std::string> arguments{executablePath(), "--probe-worker", "--iface", row.name,
                    "--domain", "0", "--timeout", std::to_string(options.timeout), "--retries", std::to_string(options.retries),
                    "--quiet-seconds", std::to_string(options.quiet_seconds)};
                if (options.check_only) arguments.push_back("--check-only");
                result = parseWorkerResult(run(arguments, options.timeout+options.retries*7+options.quiet_seconds+12));
                report["probes"].push_back(result);
                if (result["ready"].as<bool>(false)) break;
                if (attempt+1 < options.retries) { std::cout << "Probe failed; retrying DDS discovery...\n" << std::flush; pause(1.); }
            }
            for (const auto& field : result) report[field.first.as<std::string>()] = field.second;
            const bool ready = result["ready"].as<bool>(false);
            std::cout << json(result) << '\n'
                      << (ready ? "READY: network/controller ownership checks passed.\n" : "NOT READY: see report; no policy was started.\n");
            exit_code = ready ? 0 : 2;
        }
    } catch (const std::exception& error) {
        report["ready"] = false; report["error"] = stringNode(error.what());
        std::cerr << "NOT READY: " << error.what() << '\n';
    }
    try {
        std::ofstream output(path);
        if (!output) throw std::runtime_error("cannot write preparation report");
        output << json(report) << '\n';
        std::cout << "Report: " << path.string() << '\n';
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 2; }
    return exit_code;
}
}  // namespace g1prepare

#ifndef G1_PREPARE_NO_MAIN
int main(int argc, char** argv) { return g1prepare::mainImpl(argc, argv); }
#endif
