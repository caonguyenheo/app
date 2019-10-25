#!/bin/bash
echo $(date) >> /home/trailer/katri-logger/cronruntime.txt

if [ -f "/home/trailer/katri-logger/ping.txt" ]
then
    value=`cat /home/trailer/katri-logger/ping.txt`
    echo $value >> /home/trailer/katri-logger/cronruntime.txt
    
    if [ -z $value ]
    then
        systemctl restart katri.logger.service
    fi
fi

