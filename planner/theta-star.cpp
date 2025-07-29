#include "theta-star.hpp"
#include <cstddef>
#include <cstdio>
#include <functional>
#include <iostream>
#include <optional>

namespace SimplePlanner {
ThetaStar::ThetaStar() {}
ThetaStar::~ThetaStar() {}

void ThetaStar::init(AlgorithmConfig config, std::unique_ptr<Core::Logger> logger) {
  this->config = config;
  this->logger = std::move(logger);
  this->stop_thread = false;
  this->generate_travel_graph();
}

/*
 * Generates each node in the graph
 * Each node represents a voxel in space
 * */
void ThetaStar::generate_travel_graph() {
  double resolution = this->config.resolution;
  double max_distance = this->config.max_distance;
  if (resolution == 0) return;
  int total_layers = std::ceil(max_distance / resolution);
  if (total_layers == 0) return;

  this->layers.reserve(total_layers);

  // TODO: preallocate points
  size_t index = 0;
  double previous_max = 0;
  for (int i = 0; i < total_layers; i++) {
    size_t internal_layer_index = 0;
    std::vector<std::shared_ptr<PathNode>> layer;
    double max = i * resolution;
    for (double x = -max; x <= max || isEqualDouble(x, max); x += resolution) {
      double y_max = std::tan(std::acos(x / max)) * x + resolution;
      if (isEqualDouble(x, 0.0)) {
        y_max = max;
      }
      y_max = std::ceil(y_max / resolution) * resolution;
      if (!std::isnormal(y_max)) y_max = max;
      for (double y = -y_max; y <= y_max || isEqualDouble(y, y_max);
           y += resolution) {
        for (double z = -y_max; z <= y_max || isEqualDouble(z, y_max);
             z += resolution) {
          Eigen::Vector3d coords(x, y, z);
          double distance_to_origin = coords.norm();
          if (distance_to_origin > max || distance_to_origin < previous_max) {
            continue;
          }
          auto node = std::make_shared<PathNode>(coords);
          node->index = index;
          node->layer = i;
          node->internal_layer_index = internal_layer_index;
          index++;
          internal_layer_index++;
          layer.push_back(node);
          kdtree_nodes.push_back(Kdtree::KdNode(std::vector<double>{x, y, z},
                                                (void *)node.get(), index));
        }
      }
    }
    previous_max = max;
    layers.push_back(layer);
  }
  this->graph_head = layers[0][0];
  this->logger->info("Finished generating layers");
  this->kdtree = std::make_unique<Kdtree::KdTree>(&kdtree_nodes);
  Kdtree::KdNodeVector neighbors;
  for (Kdtree::KdNode &node : kdtree_nodes) {
    this->kdtree->range_nearest_neighbors(node.point, resolution * 3.0,
                                          &neighbors);
    PathNode *path_node = (PathNode *)node.data;
    std::shared_ptr<PathNode> shared_node =
        layers[path_node->layer][path_node->internal_layer_index];
    for (Kdtree::KdNode neighbor : neighbors) {
      PathNode *neighbor_data = (PathNode *)neighbor.data;
      if (neighbor_data->layer == path_node->layer + 1) {
        auto shared_neighbor =
            layers[neighbor_data->layer][neighbor_data->internal_layer_index];
        shared_node->children.push_back(std::make_pair(
            shared_neighbor,
            (shared_neighbor->coords - shared_node->coords).norm()));
      }
    }
  }
  this->logger->info("Finished generating travel graph");
}

/*
 * Includes the obstacles from the cloud point into the graph
 * */
void ThetaStar::add_octree_costs() {
  const double min_distance = this->config.min_distance;
  const double safe_distance = this->config.safe_distance;
  const double preferred_distance = this->config.preferred_distance;

  Kdtree::KdNodeVector results;
  Eigen::Vector3d obstacle;
  for (const pcl::PointXYZ &point : this->octree_points) {
    Kdtree::CoordPoint point_coord{point.x, point.y, point.z};
    this->kdtree->range_nearest_neighbors(point_coord, preferred_distance,
                                          &results);
    obstacle(0) = point.x;
    obstacle(1) = point.y;
    obstacle(2) = point.z;
    for (auto &node : results) {
      PathNode *path_node = (PathNode *)node.data;
      auto shared_node =
          this->layers[path_node->layer][path_node->internal_layer_index];
      if (shared_node->coords.norm() <= min_distance) continue;
      double distance = (obstacle - shared_node->coords).norm();
      double cost = 2.0 / distance;
      if (!std::isnormal(cost) || distance < safe_distance) {
        shared_node->isObstacle = true;
        cost = INF;
      }
      if (shared_node->cost < cost) {
        this->touched_nodes.push_back(shared_node);
        shared_node->cost = cost;
      }
    }
  }
}

void ThetaStar::add_costs() {
  if (!this->empty_cloud) {
    this->add_octree_costs();
  }

  this->add_synthetic_costs();
}

void ThetaStar::clear_nodes() {
  for (auto &node : this->kdtree->allnodes) {
    PathNode *path_node = (PathNode *)node.data;
    path_node->clear();
  }
}

/*
 * Main thread of the algotrithm
 * */
void ThetaStar::run(std::function<void(const PlanRequest &)> executing,
                    std::function<void(PlanResponse)> res) {
  this->logger->info("Running planner thread");
  while (true) {
    try {
      std::unique_lock<std::mutex> qlock(this->queue_mutex);
      this->queue_cv.wait(
          qlock, [&]() { return !this->queue.empty() || this->stop_thread; });
      if (this->stop_thread) {
        break;
      }
      if (!this->ready_to_plan()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      SimplePlanner::PlanRequest request = std::move(this->queue.front());
      this->queue.pop();
      executing(request);
      qlock.unlock();

      this->clear_nodes();
      this->add_costs();
      this->add_heuristics();
      PathNodeVector result;
      bool valid = true;
      bool require_new_route =
          request.type == RequestType::START ||
          (request.type == RequestType::REPLAN &&
           this->current_trajectory.getPoses().size() <=
               request.body.value().current_index) ||
          (request.type == RequestType::REPLAN &&
           request.body.value().path_id != this->path_sequence);

      if (require_new_route) {
        // If a nw route is required, update the sequence and plan normaly
        this->path_sequence += 1;
        this->real_goal = request.goal;
        this->last_local_goal = nullptr;
        this->logger->info("Finding local goal");
        this->find_local_goal();
        this->logger->info("Found local goal");
        if (eq(this->local_goal, pcl::PointXYZ(0, 0, 0))) {
          continue;
        }
        result = this->plan();
      } else {
        ReplanRequest replan = request.body.value();
        // Most of the next code is the logic required to "snap" the current
        // trajectory back to nodes in the graph, so the line of sight is able
        // to be checked
        auto it = replan.path.begin();
        auto end = replan.path.end();
        double max_distance = this->config.max_distance;
        std::advance(it, replan.current_index);

        Kdtree::KdNodeVector results;
        std::shared_ptr<PathNode> next_node = nullptr;
        std::shared_ptr<PathNode> node = nullptr;

        geometry_msgs::msg::Pose pose = *it;
        Kdtree::CoordPoint point_coord{pose.position.x, pose.position.y,
                                       pose.position.z};
        this->kdtree->k_nearest_neighbors(point_coord, 1, &results);
        if (results.empty()) {
          valid = false;
        }
        for (; it != end && valid; ++it) {
          PathNode *tmp_node = (PathNode *)results[0].data;
          node = this->layers[tmp_node->layer][tmp_node->internal_layer_index];
          if (node->isObstacle) {
            valid = false;
            break;
          }
          double distance = node->coords.squaredNorm();
          if (distance > max_distance) {
            valid = false;
            break;
          }
          result.push_back(node);
          auto next_it = std::next(it);
          if (next_it != end) {
            this->kdtree->k_nearest_neighbors(point_coord, 1, &results);
            if (results.empty()) {
              valid = false;
              break;
            }
            tmp_node = (PathNode *)results[0].data;
            next_node =
                this->layers[tmp_node->layer][tmp_node->internal_layer_index];
            if (!line_of_sight(node, next_node)) {
              valid = false;
              break;
            }
            node = next_node;
          }
        }

        this->path_sequence = replan.path_id;

        if (!valid) {
          result = this->plan();
          this->path_sequence++;
        }
      }

      /*
       * Send back the result
       * */
      SimplePlanner::PlanResponse msg;
      msg.request = request;
      msg.path_id = this->path_sequence;
      msg.path = result;
      res(msg);
      {
        std::lock_guard<std::mutex> lock(this->queue_mutex);
        if (this->queue.empty()) {
          this->planning = false;
        }
      }

    } catch (std::exception &e) {
      std::stringstream ss;
      ss << "Exception while planning: " << e.what();
      this->logger->error(ss.str());
    } catch (...) {
      this->logger->error("Exception while planning");
    }
  }
}

/*
 * Backtrace from the goal to the start
 * */
PathNodeVector ThetaStar::recover_path(
    const std::shared_ptr<PathNode> &final_node) {
  this->last_path.clear();
  std::shared_ptr<PathNode> current = final_node;
  while (current != NULL) {
    if (current->isObstacle) {
      throw std::runtime_error("Invalid path");
    }
    auto pathIt = this->last_path.begin();
    this->last_path.insert(pathIt, current);
    current = current->previous_position;
  }
  return this->last_path;
}

bool ThetaStar::line_of_sight(std::shared_ptr<PathNode> node1,
                              std::shared_ptr<PathNode> node2) {
  if (node1->isObstacle || node2->isObstacle) {
    return false;
  }
  Eigen::Vector3d dir = node2->coords - node1->coords;
  double distance = (node2->coords - node1->coords).norm();
  Ray ray(node1->coords, dir);

  double resolution = this->config.resolution;
  std::vector<Kdtree::kdtree_node *> queue{this->kdtree->root};
  while (!queue.empty()) {
    Kdtree::kdtree_node *node = queue[0];
    queue.erase(queue.begin());
    Eigen::Vector3d lobound(node->lobound[0], node->lobound[1],
                            node->lobound[2]);
    Eigen::Vector3d upbound(node->upbound[0], node->upbound[1],
                            node->upbound[2]);
    if (!eq(*node1, node->point)) {
      Eigen::Vector3d min(node->point[0] - resolution / 2.0,
                          node->point[1] - resolution / 2.0,
                          node->point[2] - resolution / 2.0);
      Eigen::Vector3d max(node->point[0] + resolution / 2.0,
                          node->point[1] + resolution / 2.0,
                          node->point[2] + resolution / 2.0);
      if (rayIntersects(ray, min, max)) {
        Eigen::Vector3d point(node->point[0], node->point[1], node->point[2]);
        if ((point - node1->coords).norm() > distance) continue;
        PathNode *pathNode =
            (PathNode *)this->kdtree->allnodes[node->dataindex].data;
        if (pathNode->isObstacle) return false;
      }
    }

    // If the origin is inside the space the ray obviously intersects it
    if (isInsideBox(node1->coords, lobound, upbound)) {
      if (node->loson != NULL) {
        queue.push_back(node->loson);
      }
      if (node->hison != NULL) {
        queue.push_back(node->hison);
      }
      continue;
    }

    if (rayIntersects(ray, lobound, upbound)) {
      if (node->loson != NULL) {
        queue.push_back(node->loson);
      }
      if (node->hison != NULL) {
        queue.push_back(node->hison);
      }
      continue;
    }
  }
  return true;
}

PathNodeVector ThetaStar::plan() {
  std::vector<std::shared_ptr<PathNode>> queue;
  std::vector<std::shared_ptr<PathNode>> visited;
  queue.push_back(this->graph_head);
  while (queue.size() != 0) {
    std::shared_ptr<PathNode> current = queue.front();
    pcl::PointXYZ current_point(current->coords(0), current->coords(1),
                                current->coords(2));

    // Found objective, backtrack to find path
    if (eq(current_point, this->local_goal)) {
      std::stringstream ss;
      ss << "Goal found, after " << visited.size() << " nodes, cost "
         << current->score;
      this->logger->info(ss.str());
      return this->recover_path(current);
    }

    // Expand current node
    for (auto edge : current->children) {
      auto child = edge.first;

      // check if the node has been visited
      auto node_visited = std::find(visited.begin(), visited.end(), child);
      if (node_visited != visited.end()) continue;

      std::shared_ptr<PathNode> corrected_previous = current;
      if (current->previous_position != nullptr &&
          this->line_of_sight(current->previous_position, child)) {
        corrected_previous = current->previous_position;
      }

      // Calculate the travel costo from the current node to the child
      double new_score = child->cost +
                         (child->coords - corrected_previous->coords).norm() +
                         corrected_previous->score;
      if (child->cost < 0 || edge.second < 0 || current->score < 0) {
        throw std::runtime_error("Negative cost");
      }
      if (!std::isnormal(new_score)) new_score = INF;
      // If it's not in the queue push it
      auto found = std::find(queue.begin(), queue.end(), child);
      if (found == queue.end()) {
        child->score = new_score;
        child->previous_position = corrected_previous;
        queue.push_back(child);
        continue;
      }
      // If it is in the queue check if the new score is lower and update it
      const double current_child_score = (*found)->score;
      if (new_score < current_child_score) {
        (*found)->previous_position = corrected_previous;
        (*found)->score = new_score;
      }
    }
    // Add current node to visited
    visited.push_back(current);
    queue.erase(queue.begin());

    // Sort the queue based on score plus heuristic (lower is better)
    std::sort(queue.begin(), queue.end(), [](const auto &a, const auto &b) {
      return (a->score + 2.0 * a->heuristic) < (b->score + 2.0 * b->heuristic);
    });
  }
  this->logger->info("No path found");
  throw std::runtime_error("No path found");
}

void ThetaStar::update_config(AlgorithmConfig config) { this->config = config; }

bool ThetaStar::ready_to_plan() {
  if (this->graph_head == nullptr) {
    this->logger->info("No graph head yet");
  }
  return true;
}

/*
 * Finds the most appropriate node from the graph that is closest to the goal
 * */
void ThetaStar::find_local_goal() {
  Kdtree::KdNodeVector results;
  Kdtree::CoordPoint point_coord{this->real_goal(0), this->real_goal(1),
                                 this->real_goal(2)};
  double deviation = this->config.max_distance / 5.0;
  char tries = 0;
  char max_tries = 10;
  PathNode *node = nullptr;

  this->kdtree->k_nearest_neighbors(point_coord, 1, &results);
  point_coord = results[0].point;
  node = (PathNode *)results[0].data;

  while (node == nullptr || (node->isObstacle && tries < max_tries)) {
    this->kdtree->range_nearest_neighbors(point_coord, deviation, &results);
    if (results.size() == 0) {
      throw std::runtime_error("No nearest neighbors found");
    }
    auto distance_to_goal = INF;
    for (auto &tree_node : results) {
      node = (PathNode *)tree_node.data;
      if (!node->isObstacle && node->heuristic < distance_to_goal) {
        point_coord = tree_node.point;
        distance_to_goal = node->heuristic;
      }
    }

    if (node->isObstacle) {
      tries++;
      point_coord[1] =
          this->real_goal(1) + deviation * (tries % 2 ? 1 : -1) * tries;
      point_coord[2] += deviation;
      this->kdtree->k_nearest_neighbors(point_coord, 1, &results);
      point_coord = results[0].point;
      node = (PathNode *)results[0].data;
    }
  }
  if (node->isObstacle) {
    throw std::runtime_error("No local goal found");
  }
  this->local_goal =
      pcl::PointXYZ(point_coord[0], point_coord[1], point_coord[2]);
  this->last_local_goal = this->layers[node->layer][node->internal_layer_index];
}

void ThetaStar::add_heuristics() {
  // TODO: Just update the nodes in the distance from the origin to the goal?
  for (auto &node : this->kdtree_nodes) {
    PathNode *path_node = (PathNode *)node.data;
    path_node->heuristic = (this->real_goal - path_node->coords).norm();
  }
}

void ThetaStar::update_obstacles(std::vector<Box> obstacles) {
  this->obstacles = obstacles;
}

void ThetaStar::update_octree(std::vector<pcl::PointXYZ> octree_points) {
  if (octree_points.size() == 0) {
    this->empty_cloud = true;
  }
  this->octree_points = octree_points;
}

void ThetaStar::enqueue(PlanRequest request) {
  {
    std::lock_guard<std::mutex> lock(this->queue_mutex);
    this->queue.push(std::move(request));
  }
  this->logger->info("Goal queued");
  this->queue_cv.notify_one();
}

void ThetaStar::add_synthetic_costs() {
  const double min_distance = this->config.min_distance;
  const double safe_distance = this->config.safe_distance;
  const double preferred_distance = this->config.preferred_distance;

  for (Box &obstacle : this->obstacles) {
    std::vector<Kdtree::kdtree_node *> queue{this->kdtree->root};
    Eigen::Vector3d min(obstacle.first);
    Eigen::Vector3d max(obstacle.second);
    Eigen::Vector3d bmin = min;
    min = Eigen::Vector3d(min(0) - preferred_distance,
                          min(1) - preferred_distance,
                          min(2) - preferred_distance);
    Eigen::Vector3d bmax = max;
    max = Eigen::Vector3d(max(0) + preferred_distance,
                          max(1) + preferred_distance,
                          max(2) + preferred_distance);

    while (!queue.empty()) {
      Kdtree::kdtree_node *node = queue[0];
      queue.erase(queue.begin());
      Eigen::Vector3d lobound(node->lobound[0], node->lobound[1],
                              node->lobound[2]);
      Eigen::Vector3d upbound(node->upbound[0], node->upbound[1],
                              node->upbound[2]);

      // Check if the kdtree node bounds intersect with the box
      if (boxIntersects(lobound, upbound, min, max)) {
        if (node->loson != NULL) {
          queue.push_back(node->loson);
        }
        if (node->hison != NULL) {
          queue.push_back(node->hison);
        }
      }
      Eigen::Vector3d point(node->point[0], node->point[1], node->point[2]);
      if (point.norm() <= min_distance) continue;

      // Check if the node point is inside the box
      if (isInsideBox(point, min, max)) {
        PathNode *path_node =
            (PathNode *)this->kdtree->allnodes[node->dataindex].data;
        double distance = distanceToBox(point, bmin, bmax);
        double cost = 2.0 / distance;
        if (!std::isnormal(cost) || distance < safe_distance) {
          path_node->isObstacle = true;
          cost = INF;
        }
        if (path_node->cost < cost) {
          path_node->cost = cost;
          this->touched_nodes.push_back(
              this->layers[path_node->layer][path_node->internal_layer_index]);
        }
      }
    }
  }
}

void ThetaStar::stop() {
  {
    std::unique_lock<std::mutex> lock(this->queue_mutex);
    this->stop_thread = true;
  }
  this->queue_cv.notify_one();
}

void ThetaStar::update_current_trajectory(Path::Reader path) {
  this->current_trajectory = path;
}

Path::Reader ThetaStar::get_current_trajectory() {
  return this->current_trajectory;
}

std::vector<std::vector<std::shared_ptr<PathNode>>> ThetaStar::get_layers() {
  return this->layers;
}

}  // namespace SimplePlanner
