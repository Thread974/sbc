#!/bin/bash

make
src/sbcenc -s4 -B4 sample-mono-16.au > sample-mono-16.au.sbc4
src/sbcinfo sample-mono-16.au.sbc4
src/sbcdec -f sample-mono-16.au.sbc4.au sample-mono-16.au.sbc4

mplayer sample-mono-16.au.sbc4.au
