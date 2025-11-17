insmod ./rtw88_core.ko
insmod ./rtw88_pci.ko
insmod ./rtw88_8822c.ko
insmod ./rtw88_8822ce.ko
echo 0 > /sys/class/rfkill/rfkill0/soft
#nmcli dev wifi connect "Xiaomi 14" password wasd123wasd
