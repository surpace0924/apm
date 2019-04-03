#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <sstream>

#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)

// 測定範囲の指定 [m]
const float measur_range[2][2] = {{-0.3, 0.3}, {0, 1.8}};

int pass_num = 0;                // 測定面を通過した回数（0番から始まるので注意）
float pass_point[1000][2] = {0}; // 通過点の座標
float pass_point_ave[2] = {0};   // 通過点の成分ごとの平均
float pass_point_sig[2] = {0};   // 通過点の成分ごとの分散
std::string pass_time[1000];

visualization_msgs::MarkerArray marker_array;  // 通過点
visualization_msgs::Marker measur_surface;     // 計測面
visualization_msgs::Marker statistics_surface; // 平均分散面
jsk_rviz_plugins::OverlayText text;

// 各関数宣言
#include "text.h"
#include "nowtime_gene.h"
#include "draw.h"
#include "callback.h"

int main(int argc, char **argv)
{
    // initでROSを初期化し、apm_nodeという名前をノードにつける
    // 同じ名前のノードが複数あるとだめなので、ユニークな名前をつける
    ros::init(argc, argv, "apm_node");

    // ノードハンドラの作成。ハンドラは必要時に起動される。
    ros::NodeHandle n;

    //subscriberの作成。トピック/scanを購読する。バッファ数は1000。
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    // パブリッシャの作成。トピックに対してデータを送信
    ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
    ros::Publisher measur_surface_pub = n.advertise<visualization_msgs::Marker>("measur_surface", 1);
    ros::Publisher statistics_surface_pub = n.advertise<visualization_msgs::Marker>("statistics_surface", 1);

    ros::Publisher text_pub = n.advertise<jsk_rviz_plugins::OverlayText>("text", 1);

    ros::Rate loop_rate(30);

    // 計測面のマーカーを生成
    generateMeasurSurface(measur_surface);

    while (ros::ok())
    {
        // マーカーメッセージを送信
        markers_pub.publish(marker_array);
        measur_surface_pub.publish(measur_surface);
        statistics_surface_pub.publish(statistics_surface);

        text_pub.publish(text);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}