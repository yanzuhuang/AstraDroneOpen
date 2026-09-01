// Instructions: launch gazebo, and then run this file. An image of the the random forest created can be seen in the
// paper "Real-Time Planning with Multi-Fidelity Models for Agile Flights in Unknown Environments [ICRA 2019]

// Author: Jesus Tordesillas Torres

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <Eigen/Dense>

#include <bits/stdc++.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>

#include "gazebo_cube_spawner.h"

#include <sstream>
#include <array>
#include <cstdint>
#include <random>

using gazebo_test_tools::GazeboCubeSpawner;
using ros::NodeHandle;

#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"

GazeboCubeSpawner::GazeboCubeSpawner(NodeHandle& n) : nh(n)
{
  spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}

bool GazeboCubeSpawner::spawnCube(const std::string& name, const std::string& frame_id, float x, float y, float z,
                                  float qx, float qy, float qz, float qw, float width, float height, float depth,
                                  float mass)
{
  return spawnPrimitive(name, false, frame_id, x, y, z, qx, qy, qz, qw, width, height, depth, mass);
}

bool GazeboCubeSpawner::spawnPrimitive(const std::string& name, const bool doCube, const std::string& frame_id, float x,
                                       float y, float z, float qx, float qy, float qz, float qw, float widthOrRadius,
                                       float height, float depth, float _mass)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;

  gazebo_msgs::SpawnModel spawn;
  spawn.request.model_name = name;

  // just so the variable names are shorter..
  float w = widthOrRadius;
  float h = height;
  float d = depth;

  std::stringstream _s;
  if (doCube)
  {
    _s << "<box>\
            <size>"
       << w << " " << h << " " << d << "</size>\
          </box>";
  }
  else
  {
    _s << "<cylinder>\
                <length>"
       << h << "</length>\
                <radius>" << w << "</radius>\
            </cylinder>";
  }
  std::string geometryString = _s.str();

  float mass = _mass;
  float mass12 = mass / 12.0;

  double mu1 = 500;  // 500 for PR2 finger tip. In first experiment had it on 1000000
  double mu2 = mu1;
  double kp = 10000000;  // 10000000 for PR2 finger tip
  double kd = 1;         // 100 for rubber? 1 fir OR2 finger tip

  bool do_surface = false;
  bool do_inertia = true;

  std::stringstream s;
  s << "<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"
    << name << "'>\
        <static>false</static>\
        <link name='link'>";

  // inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia
  if (do_inertia)
  {
    double xx, yy, zz;
    if (doCube)
    {
      xx = mass12 * (h * h + d * d);
      yy = mass12 * (w * w + d * d);
      zz = mass12 * (w * w + h * h);
    }
    else
    {
      xx = mass12 * (3 * w * w + h * h);
      yy = mass12 * (3 * w * w + h * h);
      zz = 0.5 * mass * w * w;
    }
    s << "<inertial>\
        <mass>"
      << mass << "</mass>\
        <inertia>\
          <ixx>" << xx << "</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>" << yy << "</iyy>\
          <iyz>0.0</iyz>\
          <izz>" << zz << "</izz>\
        </inertia>\
          </inertial>";
  }
  s << "<collision name='collision'>\
        <geometry>"
    << geometryString;
  s << "</geometry>";

  s << "<surface>\
            <contact>\
              <collide_without_contact>true</collide_without_contact>\
            </contact>\
          </surface>";

  if (do_surface)
    s << "<surface>\
            <friction>\
              <ode>\
            <mu>"
      << mu1 << "</mu>\
            <mu2>" << mu2 << "</mu2>\
            <fdir1>0.000000 0.000000 0.000000</fdir1>\
            <slip1>0.000000</slip1>\
            <slip2>0.000000</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0.000000</restitution_coefficient>\
              <threshold>100000.000000</threshold>\
            </bounce>\
            <contact>\
              <ode>\
            <soft_cfm>0.000000</soft_cfm>\
            <soft_erp>0.200000</soft_erp>\
            <kp>" << kp << "</kp>\
            <kd>" << kd << "</kd>\
            <max_vel>100.000000</max_vel>\
            <min_depth>0.001000</min_depth>\
              </ode>\
            </contact>\
        </surface>";
  s << "</collision>\
          <visual name='visual'>";
  s << "<geometry>" << geometryString;
  s << "</geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>Gazebo/Blue</name>\
            </script>\
        </material>\
          </visual>\
        <gravity>0</gravity>\
        </link>\
      </model>\
    </sdf>";

  spawn.request.model_xml = s.str();
  spawn.request.robot_namespace = "cube_spawner";
  spawn.request.initial_pose = pose;
  spawn.request.reference_frame = frame_id;

  // ROS_INFO("Resulting model: \n %s",s.str().c_str());

  // ROS_INFO("Waiting for service");
  spawn_object.waitForExistence();
  // ROS_INFO("Calling service");

  // std::cout<<spawn.request<<std::endl;

  if (!spawn_object.call(spawn))
  {
    ROS_ERROR("Failed to call service %s", SPAWN_OBJECT_TOPIC);
    return false;
  }
  ROS_INFO("Result: %s, code %u", spawn.response.status_message.c_str(), spawn.response.success);
  return spawn.response.success;
}

double randMToN(std::mt19937& random_engine, double m, double n)
{
  // mt19937 的序列由标准规定；显式归一化 engine 输出，避免依赖
  // uniform_real_distribution 或 glibc rand() 的实现细节。
  const double unit = static_cast<double>(random_engine()) /
                      (static_cast<double>(std::mt19937::max()) + 1.0);
  return m + unit * (n - m);
}

/*void generateCustomWorld(const Eigen::Vector3d& size, double density)
{

}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "forest_randomization_v1_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // voxblox::TsdfServer node(nh, nh_private);

  GazeboCubeSpawner spawner(nh);

  // ==================== 用户可调整的森林参数 ====================
  // 随机种子：供下方 std::mt19937 使用。
  // 这里的 0 只是在没有传入私有 ROS 参数 "~seed" 时使用的备用值。
  // launch_three_zone_forest.sh 会通过 FOREST_SEED 主动传入 seed，因此使用
  // 启动脚本时，应在该脚本中同步检查 forest_seed 的默认值，或这样临时指定：
  //   FOREST_SEED=2 ./launch_three_zone_forest.sh
  int seed;
  ros::param::param<int>("~seed", seed, 0);
  if (seed < 0)
  {
    ROS_FATAL("Forest seed must be non-negative, got %d", seed);
    return 2;
  }

  // 三区障碍密度，单位为“根/平方米”。
  // 当前每个区域都是 15 m × 20 m，即 300 m²；按下方 floor() 取整后，
  // 下面三个默认值会分别生成 Sparse=3 根、Medium=6 根、Dense=9 根柱子。
  // Forest Randomization v1 只随机具体布局，density 是冻结合同，不再作为运行参数覆盖。
  const double sparse_density = 0.01;
  const double medium_density = 0.02;
  const double dense_density = 0.03;

  printf("SEED=%d\n", seed);

  // v1 使用标准 mt19937。旧 glibc rand() 会让 srand(0) 与 srand(1) 得到
  // 相同序列，不满足 seed0/seed1 布局必须不同的合同，因此不再使用。
  std::mt19937 random_engine(static_cast<std::uint32_t>(seed));

  // 随机采样地图尺寸，依次为 X 长度、Y 宽度、名义 Z 高度，单位均为米。
  // 【需要同步】如果修改这里的 X 或 Y，必须同时修改 learning_speed_forest_v1_base.world：
  //   1. collision 和 visual 中的两个 <size>60 20</size>；
  //   2. 地面中心 <pose> 的 X 改为 size.x()/2，Y 必须保持 0；
  //   3. 尺寸变化较大时，再调整 GUI 相机 <pose>，保证能看到完整地图。
  // 当前地图 X 范围为 [0,60]；Y 方向以世界原点为中心，范围为 [-10,10]。
  Eigen::Vector3d size(60.0, 20.0, 5.0);  // 地图尺寸

  // Y 方向边缘留空距离，单位为米。当前 Y=[-10,10]，所以柱子中心在
  // Y=[-8,8] 内采样。X 方向的安全空地和分区范围由下方 zones 单独定义。
  Eigen::Vector3d free_space_bounds(2.0, 2.0, 2.0);

  // Forest Randomization v1 固定所有柱体高度为 5 m，中心 Z=2.5 m；
  // 半径继续保持原作者 0.25～1.0 m 的随机范围。
  const double kHeight = 5.0;
  const double kMinRadius = 0.25;
  const double kMaxRadius = 1.0;

  struct ForestZone
  {
    const char* name;
    double x_min;
    double x_max;
    double density;
  };

  // 三区的 X 范围与密度绑定关系。
  // X 坐标保持从 0 向正方向展开：地图总范围为 [0,60]。
  // 当前 X=[0,10] 是起点安全空地，X=[55,60] 是终点安全空地。
  // 下方采样位置还会按每根柱子的随机半径向区域内部收缩，确保完整柱体不会
  // 伸入相邻区域或两端安全空地。修改分区只需改这里，但必须保证各范围位于
  // [0, size.x()] 内。Y 方向是否居中由下方 position 的采样公式决定。
  const std::array<ForestZone, 3> zones = { {
      { "sparse", 10.0, 25.0, sparse_density },
      { "medium", 25.0, 40.0, medium_density },
      { "dense", 40.0, 55.0, dense_density },
  } };

  double total_volume = 0;
  int object_index = 0;
  for (const ForestZone& zone : zones)
  {
    // 每区柱子数 = floor(密度 × 该区名义面积)。
    // 只有想改变“density 如何换算为柱子数量”时，才需要修改下面这两行。
    const double zone_area = (zone.x_max - zone.x_min) * size.y();
    const int num_objects = static_cast<int>(std::floor(zone.density * zone_area));
    printf("ZONE=%s X=[%.1f,%.1f] DENSITY=%.3f OBJECTS=%d\n", zone.name, zone.x_min, zone.x_max,
           zone.density, num_objects);

    for (int i = 0; i < num_objects; ++i)
    {
      // v1 不再随机柱高；每根柱子都从 z=0 延伸到 z=5，中心固定为 z=2.5。
      const double height = kHeight;
      double radius = randMToN(random_engine, kMinRadius, kMaxRadius);
      total_volume = total_volume + 3.1415 * radius * radius * height;

      // X 坐标按柱子半径向区域内部收缩，保证完整柱体留在本区及 [0,60] 内；
      // Y 坐标以世界 Y=0 为中心，并使用 free_space_bounds.y() 保留两侧空地。
      // 保持已固化 GCC 9 world 的 RNG 消费顺序，同时消除 C++14 函数参数
      // 求值顺序未规定带来的编译器差异：既有表达式实际先采 Y、再采 X。
      const double sampled_y = randMToN(
          random_engine, -size.y() / 2.0 + free_space_bounds.y(),
          size.y() / 2.0 - free_space_bounds.y());
      const double sampled_x = randMToN(
          random_engine, zone.x_min + radius, zone.x_max - radius);
      Eigen::Vector3d position(sampled_x, sampled_y, height / 2.0);

    /*    system("rosrun gazebo_ros spawn_model -file `rospack find acl_sim`/urdf/window.urdf -urdf -x " + str(x) + " -y
       " + str(y) + " -z " + str(z) + " -R " + str(roll) + " -P " + str(pitch) + " -Y " + str(yaw) + " -model gate_" +
               str(uniform(1, 10000)));*/

      double x = position[0];
      double y = position[1];
      double z = position[2];

      const std::string model_name = std::string(zone.name) + "_" + std::to_string(object_index++);
      if (!spawner.spawnCube(model_name, "world", x, y, z, 0, 0, 0, 1, radius, height, 1, 2))
      {
        ROS_FATAL("Failed to spawn forest model %s", model_name.c_str());
        return 3;
      }

    std::string x_string = std::to_string(x);
    std::string y_string = std::to_string(y);

    std::string z_string = std::to_string(z);
    std::string i_string = std::to_string(i);

    /*    system(("rosrun gazebo_ros spawn_model -file `rospack find acl_sim`/models/cylinder/model.sdf -sdf -x " +
       x_string + " -y " + y_string + " -z 0 -model pole_" + i_string) .c_str());*/

    /*    world_.addObject(std::unique_ptr<voxblox::Object>(
            new voxblox::Cylinder(position.cast<float>(), radius, height, voxblox::Color::Gray())));*/

    }
  }

  printf("Total Volume=%f\n", total_volume);
  printf("Map area=%f\n", size.x() * size.y());
  printf("TOTAL_OBJECTS=%d HEIGHT=%.1f Z_CENTER=%.1f\n", object_index, kHeight, kHeight / 2.0);

  return 0;
}
