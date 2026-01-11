#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/color.hpp>
#include <ftxui/component/event.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <map>
#include <iomanip>
#include <csignal>
#include <set>

#include <sys/sysinfo.h>
#include <sys/statvfs.h>
#include <sys/resource.h> // For setpriority
#include <dirent.h>
#include <unistd.h>
#include <stdlib.h> // for getloadavg

using namespace ftxui;

// --- Data Structures ---
struct ProcessInfo {
    int pid;
    std::string name;
    float cpu_usage;
    float mem_usage;
    int nice; // Priority (-20 to 19)
};

struct SystemState {
    float cpu_percent = 0.0f;
    float mem_percent = 0.0f;
    float cpu_temp = 0.0f;
    float disk_percent = 0.0f;
    float net_rx_kbps = 0.0f;
    float net_tx_kbps = 0.0f;
    double load_avg[3] = {0.0, 0.0, 0.0}; // 1, 5, 15 min
    
    std::deque<int> cpu_history;
    std::deque<int> net_rx_history; 
    std::deque<int> net_tx_history; 
    
    std::vector<ProcessInfo> processes;
    std::vector<std::string> ros_node_names;
    std::vector<std::string> detected_workspaces;
    
    // Configs
    int mem_threshold = 85; 
    bool turbo_mode_active = false;

    std::string last_message = "Ready. 't': Turbo ROS | '+/-': Renice | 'm': Free Mem";
    bool ros_master_online = false;
    bool is_root = false; 
};

SystemState g_state;
std::mutex g_mutex;
bool g_running = true;

// --- Helpers ---

std::string exec_command(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "Failed";
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}

// Memory Free
bool try_free_memory(std::string& out_msg) {
    if (geteuid() != 0) {
        out_msg = "Error: Root required.";
        return false;
    }
    int ret = system("sync && echo 3 > /proc/sys/vm/drop_caches");
    if (WIFEXITED(ret) && WEXITSTATUS(ret) == 0) {
        out_msg = "Success: Cache cleared.";
        return true;
    } else {
        out_msg = "Failed: Could not clear cache.";
        return false;
    }
}

// Priority Management
bool set_proc_priority(int pid, int new_nice, std::string& out_msg) {
    if (geteuid() != 0) {
        out_msg = "Error: Root required to change priority.";
        return false;
    }
    // setpriority expects target, pid, priority
    if (setpriority(PRIO_PROCESS, pid, new_nice) == 0) {
        out_msg = "PID " + std::to_string(pid) + " nice set to " + std::to_string(new_nice);
        return true;
    }
    out_msg = "Failed to set priority.";
    return false;
}

// Turbo Mode: Boost ROS related processes
void toggle_turbo_mode(std::string& out_msg) {
    if (geteuid() != 0) {
        out_msg = "Error: Root required for Turbo Mode.";
        return;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    g_state.turbo_mode_active = !g_state.turbo_mode_active;
    
    if (!g_state.turbo_mode_active) {
        out_msg = "Turbo Mode: OFF (Priorities reset not implemented yet)";
        return; 
    }

    int boosted = 0;
    // Simple heuristic: Boost anything with "ros", "node", "slam", "nav"
    // In a real app, this should be more strict.
    for (const auto& p : g_state.processes) {
        std::string n = p.name;
        // Basic keywords common in ROS
        if (n.find("ros") != std::string::npos || 
            n.find("node") != std::string::npos ||
            n.find("slam") != std::string::npos ||
            n.find("nav") != std::string::npos ||
            n.find("controller") != std::string::npos) {
            
            setpriority(PRIO_PROCESS, p.pid, -10); // High Priority
            boosted++;
        }
    }
    out_msg = "Turbo Mode: ON! Boosted " + std::to_string(boosted) + " ROS processes.";
}

float get_cpu_usage() {
    static unsigned long long prev_user = 0, prev_nice = 0, prev_system = 0, prev_idle = 0;
    std::ifstream file("/proc/stat");
    std::string line;
    std::getline(file, line);
    std::istringstream iss(line);
    std::string cpu_label;
    unsigned long long user, nice, system, idle;
    iss >> cpu_label >> user >> nice >> system >> idle;
    unsigned long long total = (user + nice + system + idle) - (prev_user + prev_nice + prev_system + prev_idle);
    unsigned long long total_idle = idle - prev_idle;
    float usage = 0.0f;
    if (total > 0) usage = (float)(total - total_idle) / total * 100.0f;
    prev_user = user; prev_nice = nice; prev_system = system; prev_idle = idle;
    return usage;
}

float get_mem_usage() {
    struct sysinfo si;
    if (sysinfo(&si) == 0) return (float)(si.totalram - si.freeram) / si.totalram * 100.0f;
    return 0.0f;
}

float get_disk_usage() {
    struct statvfs stat;
    if (statvfs("/", &stat) != 0) return 0.0f;
    unsigned long long total = stat.f_blocks * stat.f_frsize;
    unsigned long long available = stat.f_bavail * stat.f_frsize;
    return 100.0f * (1.0f - (float)available / total);
}

float get_cpu_temp() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (file.is_open()) {
        int temp_millideg;
        file >> temp_millideg;
        return temp_millideg / 1000.0f;
    }
    return -1.0f; 
}

void update_net_usage(float& rx_kbps, float& tx_kbps) {
    static unsigned long long prev_rx = 0, prev_tx = 0;
    static auto prev_time = std::chrono::steady_clock::now();
    
    std::ifstream file("/proc/net/dev");
    if (!file.is_open()) return;

    std::string line;
    unsigned long long curr_rx = 0, curr_tx = 0;
    std::getline(file, line); std::getline(file, line); 
    while (std::getline(file, line)) {
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string iface = line.substr(0, colon);
        iface.erase(0, iface.find_first_not_of(" "));
        if (iface == "lo") continue; 
        std::istringstream iss(line.substr(colon + 1));
        unsigned long long r_bytes, r_pkt, r_err, r_drop, r_fifo, r_frame, r_comp, r_multi;
        unsigned long long t_bytes, t_pkt, t_err, t_drop, t_fifo, t_coll, t_carr, t_comp;
        if(iss >> r_bytes >> r_pkt >> r_err >> r_drop >> r_fifo >> r_frame >> r_comp >> r_multi
               >> t_bytes >> t_pkt >> t_err >> t_drop >> t_fifo >> t_coll >> t_carr >> t_comp) {
            curr_rx += r_bytes; curr_tx += t_bytes;
        }
    }
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> diff = now - prev_time;
    float seconds = diff.count();
    if (seconds > 0.1f && prev_rx > 0) { 
        rx_kbps = (curr_rx - prev_rx) / 1024.0f / seconds;
        tx_kbps = (curr_tx - prev_tx) / 1024.0f / seconds;
        prev_rx = curr_rx; prev_tx = curr_tx; prev_time = now;
    } else if (prev_rx == 0) {
        prev_rx = curr_rx; prev_tx = curr_tx; prev_time = now;
    }
}

std::vector<ProcessInfo> get_processes() {
    std::vector<ProcessInfo> procs;
    DIR* proc_dir = opendir("/proc");
    if (!proc_dir) return procs;
    struct dirent* entry;
    
    while ((entry = readdir(proc_dir)) != NULL) {
        if (!isdigit(*entry->d_name)) continue;
        int pid = std::stoi(entry->d_name);
        
        std::string stat_path = "/proc/" + std::string(entry->d_name) + "/stat";
        std::ifstream f(stat_path);
        if(!f.is_open()) continue;
        std::stringstream buffer; buffer << f.rdbuf(); 
        std::string content = buffer.str();
        f.close();

        size_t r_paren = content.find_last_of(')');
        if (r_paren == std::string::npos) continue;
        std::string name = content.substr(content.find('(') + 1, r_paren - content.find('(') - 1);
        
        std::istringstream iss(content.substr(r_paren + 2));
        char state; int ppid, pgrp, session, tty, tpgid; unsigned int flags;
        unsigned long minflt, cminflt, majflt, cmajflt, utime, stime;
        long priority, nice;
        unsigned long rss;
        std::string ignore;
        
        // Parse up to nice (index 18/19)
        iss >> state >> ppid >> pgrp >> session >> tty >> tpgid >> flags 
            >> minflt >> cminflt >> majflt >> cmajflt 
            >> utime >> stime >> ignore >> ignore >> priority >> nice >> ignore >> ignore >> rss;
        
        ProcessInfo p;
        p.pid = pid; p.name = name; 
        p.mem_usage = (float)rss * 4096 / 1024 / 1024; 
        p.cpu_usage = (float)(utime + stime);
        p.nice = (int)nice;
        
        procs.push_back(p);
    }
    closedir(proc_dir);
    
    std::sort(procs.begin(), procs.end(), [](const auto& a, const auto& b){ return a.cpu_usage > b.cpu_usage; });
    if(procs.size() > 100) procs.resize(100);
    return procs;
}

std::vector<std::string> scan_workspaces() {
    std::set<std::string> workspaces;
    DIR* proc_dir = opendir("/proc");
    if (!proc_dir) return {};
    struct dirent* entry;
    while ((entry = readdir(proc_dir)) != NULL) {
        if (!isdigit(*entry->d_name)) continue;
        std::string pid = entry->d_name;
        std::string cmdline_path = "/proc/" + pid + "/cmdline";
        std::ifstream cmd_file(cmdline_path);
        std::string cmd; if (!(cmd_file >> cmd)) { cmd_file.close(); continue; } cmd_file.close();
        if (cmd.find("bash") == std::string::npos && cmd.find("zsh") == std::string::npos && cmd.find("python") == std::string::npos) continue;

        std::string environ_path = "/proc/" + pid + "/environ";
        std::ifstream file(environ_path); if (!file.is_open()) continue;
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        size_t pos = content.find("ROS_PACKAGE_PATH=");
        if (pos != std::string::npos) {
            size_t end = content.find('\0', pos);
            std::string path_val = content.substr(pos + 17, end - (pos + 17));
            std::stringstream ss(path_val);
            std::string segment;
            while(std::getline(ss, segment, ':')) {
                if (segment.find("/opt/ros") == std::string::npos && !segment.empty()) workspaces.insert(segment);
            }
        }
    }
    closedir(proc_dir);
    return std::vector<std::string>(workspaces.begin(), workspaces.end());
}

// --- Worker ---

void worker_thread() {
    int tick = 0;
    auto last_clean_time = std::chrono::steady_clock::now();
    bool is_root = (geteuid() == 0);
    
    while (g_running) {
        float cpu = get_cpu_usage();
        float mem = get_mem_usage();
        float temp = get_cpu_temp();
        float disk = get_disk_usage();
        float net_rx = 0, net_tx = 0;
        update_net_usage(net_rx, net_tx);
        
        double loads[3];
        if (getloadavg(loads, 3) == -1) { loads[0]=0; loads[1]=0; loads[2]=0; }

        auto procs = get_processes();
        std::vector<std::string> nodes;
        std::vector<std::string> workspaces;
        bool ros_ok = false;
        
        int threshold_copy;
        { std::lock_guard<std::mutex> lock(g_mutex); threshold_copy = g_state.mem_threshold; }
        
        auto now = std::chrono::steady_clock::now();
        if (mem > threshold_copy && std::chrono::duration_cast<std::chrono::seconds>(now - last_clean_time).count() > 300) {
            std::string msg;
            if (try_free_memory(msg)) last_clean_time = now;
            std::lock_guard<std::mutex> lock(g_mutex);
            g_state.last_message = "Auto-Free: " + msg;
        }

        if (tick % 3 == 0) {
            std::string node_list = exec_command("rosnode list 2>/dev/null");
            if (!node_list.empty() && node_list.find("ERROR") == std::string::npos) {
                ros_ok = true;
                std::stringstream ss(node_list);
                std::string line;
                while(std::getline(ss, line)) if(!line.empty()) nodes.push_back(line);
            }
            workspaces = scan_workspaces();
        } else {
            std::lock_guard<std::mutex> lock(g_mutex);
            nodes = g_state.ros_node_names;
            ros_ok = g_state.ros_master_online;
            workspaces = g_state.detected_workspaces;
        }

        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_state.cpu_percent = cpu;
            g_state.mem_percent = mem;
            g_state.cpu_temp = temp;
            g_state.disk_percent = disk;
            g_state.net_rx_kbps = net_rx;
            g_state.net_tx_kbps = net_tx;
            g_state.load_avg[0] = loads[0];
            g_state.load_avg[1] = loads[1];
            g_state.load_avg[2] = loads[2];
            
            g_state.cpu_history.push_back((int)cpu);
            if(g_state.cpu_history.size() > 60) g_state.cpu_history.pop_front();
            
            g_state.net_rx_history.push_back((int)net_rx);
            if(g_state.net_rx_history.size() > 60) g_state.net_rx_history.pop_front();
            
            g_state.net_tx_history.push_back((int)net_tx);
            if(g_state.net_tx_history.size() > 60) g_state.net_tx_history.pop_front();

            g_state.processes = procs;
            g_state.ros_node_names = nodes;
            g_state.ros_master_online = ros_ok;
            g_state.detected_workspaces = workspaces;
            g_state.is_root = is_root;
        }
        tick++;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// --- UI ---

int main() {
    std::thread bg_thread(worker_thread);
    auto screen = ScreenInteractive::Fullscreen();

    // 1. Dashboard
    auto dashboard = Renderer([]() -> Element {
        std::lock_guard<std::mutex> lock(g_mutex);
        
        std::string temp_str = (g_state.cpu_temp > 0) ? std::to_string((int)g_state.cpu_temp) + "C" : "N/A";
        Color temp_color = (g_state.cpu_temp > 75) ? Color::Red : Color::Green;

        Elements left_elems;
        left_elems.push_back(text("System Resources") | bold | underlined);
        
        // Load Avg
        std::stringstream load_ss; load_ss << std::fixed << std::setprecision(2) << g_state.load_avg[0] << ", " << g_state.load_avg[1] << ", " << g_state.load_avg[2];
        left_elems.push_back(hbox(text("Load Avg: "), text(load_ss.str()) | color(Color::Yellow)));

        left_elems.push_back(text(""));
        
        // CPU
        left_elems.push_back(hbox(text("CPU Usage: "), text(std::to_string((int)g_state.cpu_percent) + "%") | bold | color(Color::Cyan)));
        left_elems.push_back(gauge(g_state.cpu_percent / 100.0) | color(Color::Cyan));
        
        // Memory
        std::string mem_val = std::to_string((int)g_state.mem_percent) + "%";
        std::string thresh_val = std::to_string(g_state.mem_threshold) + "%";
        left_elems.push_back(text(""));
        left_elems.push_back(hbox(
            text("RAM Usage: "), text(mem_val) | bold | color(Color::Magenta),
            text("  [Limit: "), text(thresh_val) | bold | color(Color::Yellow), text("]")
        ));
        left_elems.push_back(gauge(g_state.mem_percent / 100.0) | color(g_state.mem_percent > g_state.mem_threshold ? Color::Red : Color::Magenta));
        
        // Disk
        left_elems.push_back(text(""));
        left_elems.push_back(hbox(text("Disk Usage (/): "), text(std::to_string((int)g_state.disk_percent) + "%") | bold | color(Color::Blue)));
        left_elems.push_back(gauge(g_state.disk_percent / 100.0) | color(Color::Blue));
        
        left_elems.push_back(text(""));
        left_elems.push_back(separator());
        left_elems.push_back(hbox(text("CPU Temp: "), text(temp_str) | bold | color(temp_color)));
        
        auto ros_status = g_state.ros_master_online 
            ? text(" ONLINE ") | bgcolor(Color::Green) | bold 
            : text(" OFFLINE ") | bgcolor(Color::Red) | bold;
        left_elems.push_back(hbox(text("ROS Master: "), ros_status));
        
        if (g_state.turbo_mode_active) {
            left_elems.push_back(text(""));
            left_elems.push_back(text("🚀 TURBO MODE ACTIVE") | bold | blink | color(Color::Red));
        }

        Elements right_elems;
        right_elems.push_back(text("CPU Load History (60s)") | center);
        auto cpu_graph = graph([&](int width, int height) {
            std::vector<int> result(width, 0);
            int offset = (int)g_state.cpu_history.size() - width;
            for(int i=0; i<width; ++i) {
                int idx = offset + i;
                if(idx >= 0 && idx < g_state.cpu_history.size()) result[i] = g_state.cpu_history[idx] * height / 100;
            }
            return result;
        }) | color(Color::Cyan);
        right_elems.push_back(cpu_graph | flex);

        return window(text(" Dashboard "), hbox(
            vbox(std::move(left_elems)) | flex,
            separator(),
            vbox(std::move(right_elems)) | flex
        ));
    });

    // 2. Network Tab
    auto network_renderer = Renderer([]() -> Element {
        std::lock_guard<std::mutex> lock(g_mutex);
        Elements header;
        header.push_back(hbox(text("Down: ") | bold, text(std::to_string((int)g_state.net_rx_kbps) + " KB/s") | color(Color::Green)));
        header.push_back(hbox(text(" Up: ") | bold, text(std::to_string((int)g_state.net_tx_kbps) + " KB/s") | color(Color::Yellow)));
        auto net_rx_graph = graph([&](int width, int height) {
            std::vector<int> result(width, 0);
            int offset = (int)g_state.net_rx_history.size() - width;
            int max_val = 100; for(int v : g_state.net_rx_history) if(v > max_val) max_val = v;
            for(int i=0; i<width; ++i) {
                int idx = offset + i;
                if(idx >= 0 && idx < g_state.net_rx_history.size()) result[i] = g_state.net_rx_history[idx] * height / max_val;
            }
            return result;
        }) | color(Color::Green);
        return window(text(" Network Traffic "), vbox(hbox(std::move(header)) | center, net_rx_graph | flex));
    });

    // 3. Process Manager (Smart Scheduler Edition)
    int selected_proc_idx = 0;
    int proc_scroll_start = 0;
    const int VISIBLE_PROC_ROWS = 15;
    auto proc_renderer = Renderer([&]() -> Element {
        std::lock_guard<std::mutex> lock(g_mutex);
        Elements lines;
        lines.push_back(hbox(
            text("PID") | size(WIDTH, EQUAL, 8),
            text("NAME") | size(WIDTH, EQUAL, 20),
            text("CPU") | size(WIDTH, EQUAL, 8),
            text("MEM(MB)") | size(WIDTH, EQUAL, 8),
            text("NI") | size(WIDTH, EQUAL, 4)
        ) | bold | underlined);
        
        int total = g_state.processes.size();
        int end = std::min(total, proc_scroll_start + VISIBLE_PROC_ROWS);
        for (int i = proc_scroll_start; i < end; ++i) {
            const auto& p = g_state.processes[i];
            bool sel = (i == selected_proc_idx);
            
            Color nice_color = Color::White;
            if (p.nice < 0) nice_color = Color::Red; // High Prio
            if (p.nice > 0) nice_color = Color::Green; // Low Prio

            auto row = hbox(
                text(std::to_string(p.pid)) | size(WIDTH, EQUAL, 8),
                text(p.name) | size(WIDTH, EQUAL, 20),
                text(std::to_string((int)p.cpu_usage)) | size(WIDTH, EQUAL, 8),
                text(std::to_string((int)p.mem_usage)) | size(WIDTH, EQUAL, 8),
                text(std::to_string(p.nice)) | size(WIDTH, EQUAL, 4) | color(nice_color)
            );
            if (sel) row = row | inverted;
            lines.push_back(row);
        }
        return window(text(" Processes (Press +/- to Renice) "), vbox(std::move(lines)) | flex);
    });

    // 4. ROS Manager
    int selected_node_idx = 0;
    int ros_scroll_start = 0;
    const int VISIBLE_ROS_ROWS = 15;
    auto ros_renderer = Renderer([&]() -> Element {
        std::lock_guard<std::mutex> lock(g_mutex);
        Elements lines;
        if (!g_state.detected_workspaces.empty()) {
            lines.push_back(text("Workspaces:") | bold);
            for(auto& ws : g_state.detected_workspaces) lines.push_back(text(" " + ws) | color(Color::Cyan));
            lines.push_back(separator());
        }
        int total = g_state.ros_node_names.size();
        if (total == 0) lines.push_back(text("No nodes found") | center | color(Color::Red));
        else {
            int end = std::min(total, ros_scroll_start + VISIBLE_ROS_ROWS);
            for (int i = ros_scroll_start; i < end; ++i) {
                bool sel = (i == selected_node_idx);
                lines.push_back(text(g_state.ros_node_names[i]) | (sel ? inverted : nothing));
            }
        }
        return window(text(" ROS Nodes "), vbox(std::move(lines)) | flex);
    });

    // --- Controller ---
    int tab_index = 0;
    std::vector<std::string> tab_entries = { "Dashboard", "Network", "Processes", "ROS" };
    auto tab_menu = Menu(&tab_entries, &tab_index, MenuOption::HorizontalAnimated());
    auto tab_container = Container::Tab({dashboard, network_renderer, proc_renderer, ros_renderer}, &tab_index);

    auto main_container = Container::Vertical({
        Container::Horizontal({
            Renderer([&]{ return text(" ROS-Edge-Commander v0.7 ") | bold | color(Color::Cyan) | center; }),
            tab_menu,
            Renderer([&]{ return text(" | 'q' quit ") | color(Color::GrayDark); })
        }) | border,
        tab_container | flex,
        Renderer([&]{ return text(g_state.last_message) | border | color(Color::Yellow); })
    });

    auto main_renderer = Renderer(main_container, [&] { return main_container->Render(); });

    main_renderer = CatchEvent(main_renderer, [&](Event event) {
        if (event == Event::Character('q')) { screen.ExitLoopClosure()(); return true; }
        
        // Global Actions
        if (event == Event::Character('m')) {
            std::string msg; try_free_memory(msg);
            std::lock_guard<std::mutex> lock(g_mutex); g_state.last_message = msg;
            return true;
        }
        if (event == Event::Character('t')) {
            std::string msg; toggle_turbo_mode(msg);
            std::lock_guard<std::mutex> lock(g_mutex); g_state.last_message = msg;
            return true;
        }
        if (event == Event::Character(']')) {
            std::lock_guard<std::mutex> lock(g_mutex);
            if(g_state.mem_threshold < 100) g_state.mem_threshold += 5;
            g_state.last_message = "Limit set to " + std::to_string(g_state.mem_threshold) + "%";
            return true;
        }
        if (event == Event::Character('[')) {
            std::lock_guard<std::mutex> lock(g_mutex);
            if(g_state.mem_threshold > 5) g_state.mem_threshold -= 5;
            g_state.last_message = "Limit set to " + std::to_string(g_state.mem_threshold) + "%";
            return true;
        }

        // Process Renice Logic
        if (tab_index == 2) { 
             std::lock_guard<std::mutex> lock(g_mutex);
             int total = g_state.processes.size();
             int max_idx = std::max(0, total - 1);
             
             if (event == Event::ArrowUp && selected_proc_idx > 0) { 
                 selected_proc_idx--; if(selected_proc_idx < proc_scroll_start) proc_scroll_start--; 
             }
             if (event == Event::ArrowDown && selected_proc_idx < max_idx) { 
                 selected_proc_idx++; if(selected_proc_idx >= proc_scroll_start + VISIBLE_PROC_ROWS) proc_scroll_start++; 
             }
             
             // Renice
             if (event == Event::Character('-') || event == Event::Character('_')) { // Boost (Lower Nice)
                 if (selected_proc_idx < total) {
                     std::string msg;
                     int new_nice = g_state.processes[selected_proc_idx].nice - 1;
                     if (new_nice < -20) new_nice = -20;
                     if(set_proc_priority(g_state.processes[selected_proc_idx].pid, new_nice, msg))
                        g_state.processes[selected_proc_idx].nice = new_nice; // Optimistic update
                     g_state.last_message = msg;
                 }
             }
             if (event == Event::Character('+') || event == Event::Character('=')) { // De-Boost (Higher Nice)
                 if (selected_proc_idx < total) {
                     std::string msg;
                     int new_nice = g_state.processes[selected_proc_idx].nice + 1;
                     if (new_nice > 19) new_nice = 19;
                     if(set_proc_priority(g_state.processes[selected_proc_idx].pid, new_nice, msg))
                        g_state.processes[selected_proc_idx].nice = new_nice;
                     g_state.last_message = msg;
                 }
             }

             if (event == Event::Character('k') && !g_state.processes.empty()) { 
                 kill(g_state.processes[selected_proc_idx].pid, SIGTERM); g_state.last_message = "Killed PID"; 
             }
        }
        
        if (tab_index == 3) {
             std::lock_guard<std::mutex> lock(g_mutex);
             int total = g_state.ros_node_names.size();
             int max_idx = std::max(0, total - 1);
             if (event == Event::ArrowUp && selected_node_idx > 0) { selected_node_idx--; if(selected_node_idx < ros_scroll_start) ros_scroll_start--; }
             if (event == Event::ArrowDown && selected_node_idx < max_idx) { selected_node_idx++; if(selected_node_idx >= ros_scroll_start + VISIBLE_ROS_ROWS) ros_scroll_start++; }
             if (event == Event::Character('k') && !g_state.ros_node_names.empty()) { system(("rosnode kill " + g_state.ros_node_names[selected_node_idx] + " &").c_str()); g_state.last_message = "Killed Node"; }
        }
        return main_container->OnEvent(event);
    });

    std::thread refresh_thread([&screen] {
        while(g_running) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); screen.Post(Event::Custom); }
    });

    screen.Loop(main_renderer);
    g_running = false;
    if(bg_thread.joinable()) bg_thread.join();
    if(refresh_thread.joinable()) refresh_thread.join();
    return 0;
}