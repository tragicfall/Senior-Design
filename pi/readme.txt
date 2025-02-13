//////////////////////////////////////////
// Raspberry Pi 5B (16GB RAM)
//////////////////////////////////////////

ssh tlc@pi
hostname: pi
username: tlc
password: tlc

//////////////////////////////////////////
// Raspberry Pi Settings
//////////////////////////////////////////
// Memory Information:    cat /proc/meminfo
// CPU Temperature Info:  cat /sys/class/thermal/thermal_zone0/temp
// CPU Usage Information: top (press 1)
// Network Details:       ifconfig
// Network Information:   sudo nano /etc/netplan/...

//////////////////////////////////////////
// ERB Lab 208 Wifi
//////////////////////////////////////////
ssid = "skynet"
pswd = "t3rm1n4t0r"
ip   =  192.168.1.33