#include <chrono>                                               // ROS2で時間を扱うために必要
#include <memory>                                               // shared_ptrを扱うために必要
#include <queue>                                                // queueを扱うために定義
#include <cmath>

#include <rclcpp/rclcpp.hpp>                                    // rclcppのインクルード
#include <std_msgs/msg/float32.hpp>                             // ROSのstd_msgs::float32メッセージ型を扱うための宣言
#include <m5stack_msgs/msg/plot.hpp>                            // 自作Plotメッセージ型を扱うための宣言

using namespace std::placeholders;                              // bind用の定義
using namespace std::chrono_literals;

// Plotトピック作成ノードクラス
class M5StackPlot : public rclcpp::Node                         // Nodeを継承します
{
public:
    M5StackPlot( std::string node_name )// コンストラクタ(ノード名)
     : Node(node_name)
    {
	// キューの初期化
	for( uint32_t ii=0 ; ii<320 ; ++ii ){
		plot_data_.push(0);
	}
	// Pub/Subの初期化
        sub_ = this->create_subscription<std_msgs::msg::Float32>(// Subscriber作成
            "test_sin",                                         // 購読するトピック名設定
            10,                                                 // 購読キューの数
            std::bind( &M5StackPlot::callback, this, _1) );     // メッセージ受信で呼び出す関数設定
        pub_ = this->create_publisher<m5stack_msgs::msg::Plot>("plot", 1 );
    }
    // Subコールバック
    void callback(const std_msgs::msg::Float32::SharedPtr msg){ // コールバック関数
        m5stack_msgs::msg::Plot pub_data;
        char* dst = (char*)pub_data.name.data();
        std::snprintf( dst, sizeof(pub_data.name), "Data");

        // キューのデータを入れ替え
        plot_data_.pop();
        plot_data_.push(msg->data);
        // データ更新
        auto tmp_data = plot_data_;
        for( uint32_t ii=0 ; ii<320 ; ++ii ){
            pub_data.data[ii] = (tmp_data.front() * 30000);
            tmp_data.pop();
        }
        // Publish
        pub_->publish( pub_data );
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;// Subscriberのポインタ
    rclcpp::Publisher<m5stack_msgs::msg::Plot>::SharedPtr pub_;
    std::queue<float_t> plot_data_;
};

// Plotテストクラス
class PlotTest : public rclcpp::Node                           // Nodeを継承します
{
public:
    PlotTest()// コンストラクタ(ノード名)
     : Node("plot_test_pub")
    {
        test_seed_ = 0.0;
        pub_ = this->create_publisher<std_msgs::msg::Float32>("/test_sin", 10 );   // NodeにPublisherを作成(トピック名, QoSの履歴数)
        timer_ = this->create_wall_timer(11ms, std::bind(&PlotTest::callback, this));
    }
    void callback()
    {
        auto pub_data = std_msgs::msg::Float32();
        test_seed_ += 0.05;
        pub_data.data = sin(test_seed_);
        pub_->publish(pub_data);
    }
private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float_t test_seed_;
};

