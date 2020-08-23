#!/bin/sh -x

cp /dev/null tftfonts.h

for i in tiny mid big; do
  bin2c -n $i"font" "Create_"$i"font/FontFiles/"*.vlw > $i"font.h"
  cat $i"font.h" >> tftfonts.h
done

perl -p -i -e 's/unsigned char (.*)=/const uint8_t $1 PROGMEM =/' tftfonts.h

