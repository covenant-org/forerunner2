#!/bin/bash

cd vendor
cmake -Bbuild && cmake --build -j$(nproc) build && sudo cmake --install build
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuildmav -SMAVSDK && sudo cmake --build buildmav -j$(nproc) --target install && sudo ldconfig
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuildpcl -Spcl && sudo cmake --build buildpcl -j$(nproc) && sudo cmake --install buildpcl
