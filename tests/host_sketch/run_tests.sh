#!/bin/sh
# Baut sketch.ino mit Ersatz-Headern für den PC und führt die Simulation aus.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -Istubs -o /tmp/watchmen_sketch_test test_sketch.cpp
/tmp/watchmen_sketch_test
