#include <zmq.hpp>
#include <iostream>

int main(){
    zmq::context_t ctx;
    zmq::socket_t sock(ctx, zmq::socket_type::req);
    sock.connect("tcp://127.0.0.1:4020");
    std::cout << "Connected" << std::endl;
    sock.send(zmq::str_buffer("hola"), zmq::send_flags::none);
    sock.close();
    return 0;
}