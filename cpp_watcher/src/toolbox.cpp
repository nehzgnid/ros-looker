#include "toolbox.hpp"
#include <array>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>

namespace Toolbox {

    std::string exec_cmd(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        // Merge stderr into stdout to capture errors
        std::string full_cmd = std::string(cmd) + " 2>&1";
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(full_cmd.c_str(), "r"), pclose);
        if (!pipe) {
            return "popen() failed!";
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    bool is_root() {
        return geteuid() == 0;
    }

    // --- Category: General System ---
    std::string get_uptime() {
        return exec_cmd("uptime -p");
    }

    std::string get_kernel_info() {
        return exec_cmd("uname -sr");
    }
    
    std::string get_cpu_model_name() {
        return exec_cmd("grep -m1 'model name' /proc/cpuinfo | cut -d: -f2");
    }

    std::string detect_zombies() {
        // ps aux | grep 'Z'
        std::string out = exec_cmd("ps aux | awk '$8==\"Z\" {print $2, $11}'");
        if (out.empty()) return "No zombie processes found.";
        return "PID CMD\n" + out;
    }

    std::string check_failed_services() {
        return exec_cmd("systemctl list-units --state=failed --no-pager --plain | head -n 5");
    }

    std::string list_cron_jobs() {
        // List for current user
        return exec_cmd("crontab -l | grep -v '^#' | head -n 10");
    }

    std::string get_battery_status() {
        // Try upower first
        std::string upower = exec_cmd("upower -i $(upower -e | grep 'BAT') | grep -E 'state|percentage|time to empty'");
        if (!upower.empty()) return upower;
        
        // Fallback to /sys/class/power_supply
        std::string cap = exec_cmd("cat /sys/class/power_supply/BAT0/capacity 2>/dev/null");
        std::string status = exec_cmd("cat /sys/class/power_supply/BAT0/status 2>/dev/null");
        if (!cap.empty()) return "BAT0: " + cap + "% (" + (status.empty() ? "?" : status) + ")";
        
        return "No Battery Detected";
    }

    std::string get_sensors_summary() {
        // Try 'sensors' command
        std::string s = exec_cmd("sensors | grep -E 'Package id 0:|Core 0:|fan1:' | cut -c 1-50");
        if (!s.empty()) return s;
        return "Sensors command not available";
    }


    // --- Category: Network Advanced ---
    std::string get_public_ip() {
        // Timeout 2s
        return exec_cmd("timeout 2 curl -s ifconfig.me");
    }

    std::string get_local_ips() {
        return exec_cmd("ip -4 addr show | grep -oP '(?<=inet\\s)\\d+(\\.\\d+){3}' | grep -v '127.0.0.1'");
    }

    std::string check_listening_ports() {
        return exec_cmd("ss -ltn | awk '{print $4}' | cut -d':' -f2 | sort -n | uniq | tr '\n' ' '");
    }

    std::string check_dns_resolution(const std::string& domain) {
        std::string cmd = "getent hosts " + domain + " | awk '{print $1}'";
        std::string res = exec_cmd(cmd.c_str());
        if (res.empty()) return "Resolution Failed";
        return res;
    }

    // --- Category: Security ---
    std::string check_firewall_status() {
        if (is_root()) return exec_cmd("ufw status");
        return "Requires Root (ufw)";
    }

    std::string list_sudo_users() {
        return exec_cmd("grep '^sudo:.*$' /etc/group | cut -d: -f4");
    }

    std::string count_failed_ssh_logins() {
        if (!is_root()) return "Requires Root to read auth.log";
        // Attempt to read typical ubuntu auth log
        return exec_cmd("grep 'Failed password' /var/log/auth.log 2>/dev/null | wc -l");
    }

    std::string get_last_reboots() {
        return exec_cmd("last reboot | head -n 3");
    }


    // --- Category: Disk Advanced ---
    
    std::vector<DiskInfo> get_disk_space() {
        std::string output = exec_cmd("df -h | grep -vE 'tmpfs|udev|loop'");
        std::stringstream ss(output);
        std::string line;
        std::vector<DiskInfo> disks;
        
        while (std::getline(ss, line)) {
            std::stringstream ls(line);
            DiskInfo d;
            // Handle parsing logic same as before
            ls >> d.filesystem >> d.size >> d.used >> d.avail >> d.use_percent >> d.mount;
            if (!d.mount.empty()) disks.push_back(d);
        }
        return disks;
    }

    std::string get_inode_usage() {
        return exec_cmd("df -i / | awk 'NR==2 {print $5}'");
    }

    std::string get_specific_dir_sizes() {
        // Check home, var/log, tmp
        return exec_cmd("du -sh /home /var/log /tmp 2>/dev/null");
    }

    std::string check_apt_cache_size() {
        return exec_cmd("du -sh /var/cache/apt/archives 2>/dev/null");
    }

    std::string find_large_files(const std::string& path, int size_mb) {
        std::string cmd = "find " + path + " -type f -size +" + std::to_string(size_mb) + "M -exec ls -lh {} \\; 2>/dev/null | awk '{print $5, $9}' | head -n 10";
        return exec_cmd(cmd.c_str());
    }

    // --- Category: User ---

    std::vector<UserInfo> get_users() {
        std::vector<UserInfo> users;
        std::ifstream file("/etc/passwd");
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string segment;
            std::vector<std::string> parts;
            while(std::getline(ss, segment, ':')) {
                parts.push_back(segment);
            }
            if (parts.size() >= 7) {
                try {
                    int uid = std::stoi(parts[2]);
                    if (uid >= 1000 || uid == 0) {
                        users.push_back({parts[0], parts[2], parts[3], parts[5], parts[6]});
                    }
                } catch (...) {}
            }
        }
        return users;
    }

    std::string get_login_history(int lines) {
        return exec_cmd(("last -n " + std::to_string(lines)).c_str());
    }

    std::string add_user(const std::string& username) {
        if (!is_root()) return "Error: Root required";
        if (username.empty()) return "Error: Empty username";
        std::string cmd = "useradd -m " + username;
        return exec_cmd(cmd.c_str());
    }
    
    std::string del_user(const std::string& username) {
        if (!is_root()) return "Error: Root required";
        if (username.empty()) return "Error: Empty username";
        std::string cmd = "userdel -r " + username;
        return exec_cmd(cmd.c_str());
    }

    // --- Category: DevOps ---

    std::string check_docker_status() {
        // Count running containers
        std::string res = exec_cmd("docker ps -q 2>/dev/null | wc -l");
        if (res.find("0") != std::string::npos && exec_cmd("which docker").empty()) {
             return "Docker not installed";
        }
        return "Running Containers: " + res;
    }

    std::string list_usb_devices() {
        return exec_cmd("lsusb | cut -d: -f2- | cut -d' ' -f2- | head -n 5");
    }

    std::string get_ros_env_vars() {
        // Since this runs in a shell, it might not see all vars unless exported globally or passed to the process.
        // We try to print current env vars of this process.
        std::string out = "";
        const char* master = getenv("ROS_MASTER_URI");
        const char* ip = getenv("ROS_IP");
        const char* distro = getenv("ROS_DISTRO");
        
        if (master) out += "ROS_MASTER_URI=" + std::string(master) + "\n";
        else out += "ROS_MASTER_URI=(unset)\n";
        
        if (ip) out += "ROS_IP=" + std::string(ip) + "\n";
        else out += "ROS_IP=(unset)\n";

        if (distro) out += "ROS_DISTRO=" + std::string(distro);
        
        return out;
    }

    std::string check_git_status(const std::string& path) {
        std::string cmd = "cd " + path + " && git status --short 2>&1";
        return exec_cmd(cmd.c_str());
    }

    // --- Category: AI & Deep Learning ---

    GPUInfo get_gpu_status() {
        GPUInfo info;
        // Check nvidia-smi presence
        std::string check = exec_cmd("which nvidia-smi");
        if (check.empty()) return info;

        // Query: name, utilization.gpu, memory.used, memory.total, temperature.gpu, power.draw
        std::string cmd = "nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw --format=csv,noheader,nounits";
        std::string out = exec_cmd(cmd.c_str());
        if (out.empty() || out.find("failed") != std::string::npos) return info;

        std::stringstream ss(out);
        std::string segment;
        std::vector<std::string> parts;
        while(std::getline(ss, segment, ',')) {
            parts.push_back(segment);
        }

        if (parts.size() >= 6) {
            info.available = true;
            info.name = parts[0];
            info.util = parts[1];
            info.mem_used = parts[2];
            info.mem_total = parts[3];
            info.temp = parts[4];
            info.power = parts[5];
        }
        return info;
    }

    std::string get_cuda_version() {
        std::string nvcc = exec_cmd("nvcc --version | grep release | cut -d',' -f2 | awk '{print $2}'");
        if (!nvcc.empty()) return "NVCC: " + nvcc;
        
        // Fallback to file
        std::string file_ver = exec_cmd("cat /usr/local/cuda/version.txt 2>/dev/null");
        if (!file_ver.empty()) return "File: " + file_ver;
        
        return "Not found";
    }

    std::string get_python_info() {
        std::string ver = exec_cmd("python3 --version 2>&1");
        if (ver.empty()) return "Python3 not found";
        // Remove newline
        if (!ver.empty() && ver.back() == '\n') ver.pop_back();
        
        std::string pip_count = exec_cmd("pip3 list 2>/dev/null | wc -l");
        if (pip_count.empty()) pip_count = "0";
        else if (pip_count.back() == '\n') pip_count.pop_back();

        return ver + " | Pip Packages: " + pip_count;
    }

    std::string check_dl_frameworks() {
        // Simple check if import torch/tensorflow works (slow, run async)
        std::string res = "";
        std::string torch = exec_cmd("python3 -c 'import torch; print(torch.__version__)' 2>/dev/null");
        if (!torch.empty() && torch.back() == '\n') torch.pop_back();
        res += "PyTorch: " + (torch.empty() ? "Not found" : torch) + "\n";
        
        std::string tf = exec_cmd("python3 -c 'import tensorflow; print(tensorflow.__version__)' 2>/dev/null");
        if (!tf.empty() && tf.back() == '\n') tf.pop_back();
        res += "TensorFlow: " + (tf.empty() ? "Not found" : tf);
        
        return res;
    }

    std::string list_conda_envs() {
        std::string out = exec_cmd("conda env list 2>/dev/null | grep -v '^#' | head -n 5");
        if (out.empty()) return "Conda not found or empty.";
        return out;
    }


    // --- Misc ---

    std::string get_net_connections() {
        return exec_cmd("ss -tu state established | head -n 15");
    }

    std::string check_port(const std::string& host, int port) {
        std::string cmd = "timeout 1 bash -c 'cat < /dev/null > /dev/tcp/" + host + "/" + std::to_string(port) + "' 2>&1 && echo 'Open' || echo 'Closed/Timeout'";
        return exec_cmd(cmd.c_str());
    }

    std::string test_network_speed() {
        return exec_cmd("ping -c 3 8.8.8.8 | grep 'rtt'");
    }
    
    std::string backup_file(const std::string& source, const std::string& dest) {
        if (source.empty() || dest.empty()) return "Invalid paths";
        std::string cmd = "cp -r " + source + " " + dest + " && echo 'Backup Success' || echo 'Backup Failed'";
        return exec_cmd(cmd.c_str());
    }
    
    std::string db_backup_info() {
        return "Usage: mysqldump -u root -p [db] > backup.sql";
    }

}