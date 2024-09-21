cat /dev/mcu-gpio4 | while read line; do
    echo "$(date +%s.%6N) $line"
done