cmake_minimum_required(VERSION 3.22)


project("vendor")
add_subdirectory("libzmq")
add_subdirectory("capnproto")
add_subdirectory("cppzmq")
add_subdirectory("eigen")
add_subdirectory("flann")
add_subdirectory("argparse")
add_subdirectory("yaml-cpp")
