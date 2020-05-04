#include "m5stack_example/plot_example.hpp"

int main ( int argc, char* argv[])
{
  rclcpp::init( argc, argv );

  rclcpp::executors::SingleThreadedExecutor exec;   // シングルスレッドExecutor

  auto sub_node = std::make_shared<M5StackPlot>("m5stack_plot_pub");// ノード作成
  auto pub_node = std::make_shared<PlotTest>();     // ノード作成

  exec.add_node( sub_node );                        // ノードをExecutorに登録
  exec.add_node( pub_node );                        // ノードをExecutorに登録
  exec.spin();                                      // Executorのspinを実行

  rclcpp::shutdown();

  return 0 ;
}
