#!/bin/bash

echo "===== 开始重置 WiFi ====="

# 1. 停止 hostapd
echo "[1/5] 停止 hostapd..."
killall -9 hostapd 2>/dev/null || true

# 2. 清除 wlan0 的 IP 地址
echo "[2/5] 清除 wlan0 地址..."
ip addr flush dev wlan0
sleep 0.5

# 3. 重启 wlan0 网卡
echo "[3/5] 重启 wlan0..."
ifconfig wlan0 down
sleep 1
ifconfig wlan0 up

# 4. 重启 wpa_supplicant
echo "[4/5] 重启 wpa_supplicant..."
systemctl unmask wpa_supplicant
systemctl restart wpa_supplicant

# 5. 重新加载 aic8800_fdrv 驱动
echo "[5/5] 重新加载 aic8800_fdrv 驱动..."
rmmod aic8800_fdrv 2>/dev/null || true
sleep 1
modprobe aic8800_fdrv

echo "===== WiFi 重置完成 ====="
