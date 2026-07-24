#!/bin/sh
set -ex

cd /mnt/storage/
for i in $(seq 20 999); do
  read
  (
    printf "2\n"
    sleep 1
    printf "0\n"
    sleep 1
    printf "1\n"
    sleep 1
    printf "255\n"
  ) | /mnt/system/usr/bin/sample_sensor_test > /dev/null 2>&1
  mv sample_0.yuv shot_$(printf %03d $i).yuv
done
