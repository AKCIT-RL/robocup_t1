#!/bin/bash
cd `dirname $0`
bash sync_time.sh
sudo systemctl restart booster-daemon-perception
sudo systemctl restart booster-vision-data.service
