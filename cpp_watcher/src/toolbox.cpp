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