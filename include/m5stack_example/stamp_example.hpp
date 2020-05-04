#include <chrono>                                               // ROS2で時間を扱うために必要
#include <memory>                                               // shared_ptrを扱うために必要
#include <cstring>

#include <rclcpp/rclcpp.hpp>                                    // rclcppのインクルード
#include <m5stack_msgs/msg/stamp_request.hpp>                   // 自作StampRequestメッセージ型
#include <m5stack_msgs/msg/stamp_data.hpp>                      // 自作StampDataメッセージ型
#include <m5stack_example/stamp_a.h>                            // スタンプAのXBM形式データ
#include <m5stack_example/stamp_b.h>                            // スタンプBのXBM形式データ
#include <m5stack_example/stamp_c.h>                            // スタンプCのXBM形式データ

using namespace std::placeholders;                              // bind用の定義
using namespace std::chrono_literals;

// Plotトピック作成ノードクラス
class M5StackStamp : public rclcpp::Node                        // Nodeを継承します
{
public:
    M5StackStamp( std::string node_name )// コンストラクタ(ノード名)
     : Node(node_name)
     , total_(8)                                                // 8分割
     , length_(900)                                             // 1送信あたり900Byte
     , size_((240/8)*240)                                       // XBMデータのサイズ
     , bits_( {&stamp_a_bits[0],&stamp_b_bits[0],&stamp_c_bits[0]} )// XBMデータの配列登録
    {
        // Pub/Subの初期化
        sub_ = this->create_subscription<m5stack_msgs::msg::StampRequest>(// 画像要求Subscriber作成
            "stamp/request",                                    // 購読するトピック名設定
            1,                                                  // 購読キューの数
            std::bind( &M5StackStamp::callback, this, _1) );    // メッセージ受信で呼び出す関数設定
        pub_ = this->create_publisher<m5stack_msgs::msg::StampData>("stamp/data", 1 );
                                                                // 画像Publisher作成
    }
    // Subコールバック
    void callback(const m5stack_msgs::msg::StampRequest::SharedPtr msg)// コールバック関数
    {
        m5stack_msgs::msg::StampData pub_data;                  // 画像送信メッセージ
        uint16_t index;

        pub_data.seq = msg->seq;                                // シーケンス番号格納
        pub_data.total = total_;                                // 分割数格納
        pub_data.length = length_;                              // 送信長格納
        pub_data.index = msg->index;                            // 分割番号格納
        if( msg->type < type_num_ ){                            // 要求スタンプ種別が有効か
            index = msg->index;                                 // 分割番号
            memcpy( &pub_data.data[0], &(bits_[msg->type])[ index2addr(index, size_) ], length_ );
                                                                // 画像データ格納
        }else{
            // Stamp種別エラー
            pub_data.total = 0;
            pub_data.length = 0;
        }
        // Publish
        pub_->publish( pub_data );
    }
    // データインデックスから配列アドレスを求める関数
    uint16_t index2addr( uint16_t index, int size )
    {
        uint16_t result = 0;
        
        if( index < total_ ){
            result = (index * (size / total_));
        }
        return result;
    }
private:
    rclcpp::Subscription<m5stack_msgs::msg::StampRequest>::SharedPtr sub_;// Subscriberのポインタ
    rclcpp::Publisher<m5stack_msgs::msg::StampData>::SharedPtr pub_;// Publisherのポインタ
    constexpr static int type_num_ = 3;                         // スタンプ種別数
    const uint16_t total_, length_, size_;
    const std::array<uint8_t*, type_num_> bits_;
};
