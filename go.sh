#!/bin/sh

make
src/sbcenc  -b26 -B16 -s8   sample.au > sample.au.msbc
src/sbcinfo sample.au.msbc
src/sbcdec  -f sample.au.msbc.au sample.au.msbc
mplayer sample.au.msbc.au
