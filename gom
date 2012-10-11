#!/bin/sh

make
src/sbcenc  -m -b26 -B16 -s8   sample.au > sample.au.msbc
src/sbcinfo sample.au.msbc
src/sbcdec  -m -f sample.au.msbc.au sample.au.msbc
mplayer sample.au.msbc.au
