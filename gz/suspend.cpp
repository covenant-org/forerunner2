#include "argument_parser.hpp"
#include "suspend.hpp"
#include <cmath>
#include <gz/msgs/pointcloud.pb.h>
#include <gz/transport/MessageInfo.hh>
#include <gz/transport/Node.hh>

GZSuspend::GZSuspend(Core::ArgumentParser args) : Core::Vertex(args) {
  _gz_node = std::make_shared<gz::transport::Node>();
}

void GZSuspend::run() {
  // Crear publisher
  std::string topic = "/world/default/wrench";
  auto pub = this->_gz_node->Advertise<gz::msgs::EntityWrench>(topic);

  // Esperar a que se establezca la conexión
  this->_logger.info("Esperando conexión con Gazebo...");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (!pub.HasConnections()) {
    this->_logger.error("No se pudo conectar al topic: ");
    return;
  }

  this->_logger.info("Conectado! Obteniendo masa del modelo...");

  // Obtener la masa del link desde Gazebo
  double mass = 1.067;
  bool massReceived = false;

  // Crear mensaje de solicitud de estado
  gz::msgs::Entity entityMsg;
  entityMsg.set_name(_args.get_argument("--model") +
                     "::" + _args.get_argument("--link"));
  entityMsg.set_type(gz::msgs::Entity::LINK);

  // Callback para recibir la información del link
  std::function<void(const gz::msgs::Link&)> cb =
      [&](const gz::msgs::Link& msg) -> void {
    this->_logger.debug("State callback");
    if (msg.has_inertial()) {
      mass = msg.inertial().mass();
      massReceived = true;
      std::cout << "Masa obtenida: " << mass << " kg" << std::endl;
    }
  };

  // Suscribirse al topic de estado del link
  std::string stateTopic = "/world/default/model/" +
                           _args.get_argument("--model") + "/link/" +
                           _args.get_argument("--link") + "/state";
  _gz_node->Subscribe(stateTopic, cb);

  // Esperar a recibir la masa (timeout de 5 segundos)

  // Gravedad (m/s²)
  double gravity = 9.81;

  // Fuerza necesaria para contrarrestar la gravedad
  double force_z = mass * gravity;

  this->_logger.info("Aplicando fuerza de levitación: %f", force_z);

  // Bucle principal para mantener el objeto suspendido
  while (true) {
    // Crear mensaje de fuerza
    gz::msgs::EntityWrench wrenchMsg;

    // Establecer la entidad (modelo y link)
    wrenchMsg.mutable_entity()->set_name(_args.get_argument("--model") +
                                         "::" + _args.get_argument("--link"));
    wrenchMsg.mutable_entity()->set_type(gz::msgs::Entity::LINK);

    // Configurar la fuerza en el eje Z (hacia arriba)
    gz::msgs::Wrench* wrench = wrenchMsg.mutable_wrench();
    gz::msgs::Vector3d* force = wrench->mutable_force();
    force->set_x(0.0);
    force->set_y(0.0);
    force->set_z(force_z);  // Fuerza hacia arriba

    // No aplicamos torque
    gz::msgs::Vector3d* torque = wrench->mutable_torque();
    torque->set_x(0.0);
    torque->set_y(0.0);
    torque->set_z(0.0);

    // Publicar el mensaje
    pub.Publish(wrenchMsg);

    // Esperar un poco antes de la siguiente actualización
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

int main(int argc, char** argv) {
  Core::BaseArgumentParser args(argc, argv);
  args.add_argument("--model").help("Model name").required();
  args.add_argument("--link").help("Link name").required();
  auto gz = std::make_shared<GZSuspend>(args);
  gz->run();
  return 0;
}
