#include "vertex.hpp"
#include <pcl/impl/point_types.hpp>

class Mapping : public Core::Vertex {
 private:
  std::shared_ptr<Core::Subscriber<PointCloud>> _sub;
  std::shared_ptr<Core::Subscriber<Odometry>> _odom_sub;
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> _cloud;

 public:
  void stich_cloud(const Core::IncomingMessage<PointCloud> &);
  Mapping(Core::ArgumentParser);
}
