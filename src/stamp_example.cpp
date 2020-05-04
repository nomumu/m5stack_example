#include "m5stack_example/stamp_example.hpp"

int main ( int argc, char* argv[])
{
  rclcpp::init( argc, argv );

  rclcpp::executors::SingleThreadedExecutor exec;   // シングルスレッドExecutor

  auto stamp_node = std::make_shared<M5StackStamp>("m5stack_stamp_host");// ノード作成

  exec.add_node( stamp_node );                      // ノードをExecutorに登録
  exec.spin();                                      // Executorのspinを実行

  rclcpp::shutdown();

  return 0 ;
}
