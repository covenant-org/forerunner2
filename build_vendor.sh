#!/bin/bash

set -e

cd vendor
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -DCPPZMQ_BUILD_TESTS=OFF && cmake --build build -j$(nproc) && sudo cmake --install build
cmake -DCMAKE_BUILD_TYPE=Release -Bbuildarg -Sargparse && sudo cmake --build buildarg -j$(nproc) --target install && sudo ldconfig
cmake -DCMAKE_BUILD_TYPE=Release -Bbuildyaml -Syaml-cpp && sudo cmake --build buildyaml -j$(nproc) --target install
cmake -DCMAKE_BUILD_TYPE=Release -Bbuildmav -SMAVSDK && sudo cmake --build buildmav -j$(nproc) --target install && sudo ldconfig
cmake -DCMAKE_BUILD_TYPE=Release -Bbuildpcl -Spcl && sudo cmake --build buildpcl && sudo cmake --install buildpcl
