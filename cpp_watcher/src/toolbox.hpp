#ifndef TOOLBOX_HPP
#define TOOLBOX_HPP

#include <string>
#include <vector>
#include <tuple>

namespace Toolbox {

    // --- Helpers ---
    std::string exec_cmd(const char* cmd);
    bool is_root();

    // --- Category: General System ---
    std::string get_uptime();
    std::string get_kernel_info();
    std::string get_cpu_model_name();
    std::string detect_zombies(); // Returns count and list
    std::string check_failed_services(); // systemctl --failed
    std::string list_cron_jobs();

    // --- Category: Network Advanced ---
    std::string get_public_ip(); // curl
    std::string get_local_ips(); // ip addr
    std::string check_listening_ports(); // ss -ltn
    std::string check_dns_resolution(const std::string& domain = "google.com");
    
    // --- Category: Security ---
    std::string check_firewall_status(); // ufw/iptables
    std::string list_sudo_users();
    std::string count_failed_ssh_logins(); // grep auth.log
    std::string get_last_reboots();

    // --- Category: Disk Advanced ---
    struct DiskInfo {
        std::string filesystem;
        std::string size;
        std::string used;
        std::string avail;
        std::string use_percent;
        std::string mount;
    };
    std::vector<DiskInfo> get_disk_space();
    std::string get_inode_usage();
    std::string get_specific_dir_sizes(); // /home, /var/log, /tmp
    std::string check_apt_cache_size();
    std::string find_large_files(const std::string& path, int size_mb);
    
    // --- Category: User ---
    struct UserInfo {
        std::string name;
        std::string uid;
        std::string gid;
        std::string home;
        std::string shell;
    };
    std::vector<UserInfo> get_users();
    std::string get_login_history(int lines);
    std::string add_user(const std::string& username); 
    std::string del_user(const std::string& username);

    // --- Category: DevOps (Docker/ROS/HW) ---
    std::string check_docker_status();
    std::string list_usb_devices();
    std::string get_ros_env_vars(); // ROS_MASTER_URI, etc.
    std::string check_git_status(const std::string& path);

    // --- Legacy/Misc ---
    std::string get_net_connections();
    std::string check_port(const std::string& host, int port);
    std::string test_network_speed();
    std::string backup_file(const std::string& source, const std::string& dest);
    std::string db_backup_info(); 
}

#endif