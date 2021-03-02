#!/usr/bin/bash

export LD_LIBRARY_PATH=/data/data/com.termux/files/usr/lib
export HOME=/data/data/com.termux/files/home
export PATH=/usr/local/bin:/data/data/com.termux/files/usr/bin:/data/data/com.termux/files/usr/sbin:/data/data/com.termux/files/usr/bin/applets:/bin:/sbin:/vendor/bin:/system/sbin:/system/bin:/system/xbin:/data/data/com.termux/files/usr/bin/python
export PYTHONPATH=/sdcard/

cd /sdcard/
echo "downloading osm apk"
wget https://raw.githubusercontent.com/rav4kumar/openpilot/577b9a0cdeacf2f4b1d8ba357e09cf32a49178c6/apk/net.osmand.plus.apk
echo "installing the apk"
pm install -r "/sdcard/net.osmand.plus.apk";
echo "Going to sleep because i am tired."
sleep 5;
