#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <math.h>

#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)

// 測定範囲の指定 [m]
const float measur_range[2][2] = {{-0.3, 0.3}, {0, 1.8}};

int pass_num = 0;                // 測定面を通過した回数（0番から始まるので注意）
float pass_point[1000][2] = {0}; // 通過点の座標

visualization_msgs::MarkerArray marker_array;

void addMarkerArray(visualization_msgs::MarkerArray &ma, int i, float x, float y)
{
    ma.markers[i].header.frame_id = "/laser_frame";
    ma.markers[i].header.stamp = ros::Time::now();
    ma.markers[i].ns = "basic_shapes";

    ma.markers[i].type = visualization_msgs::Marker::SPHERE;
    ma.markers[i].action = visualization_msgs::Marker::ADD;
    ma.markers[i].id = i;
    ma.markers[i].lifetime = ros::Duration();

    ma.markers[i].scale.x = 0.05;
    ma.markers[i].scale.y = 0.05;
    ma.markers[i].scale.z = 0.05;
    ma.markers[i].pose.position.x = x;
    ma.markers[i].pose.position.y = y;
    ma.markers[i].pose.position.z = 0;
    ma.markers[i].pose.orientation.x = 0;
    ma.markers[i].pose.orientation.y = 0;
    ma.markers[i].pose.orientation.z = 0;
    ma.markers[i].pose.orientation.w = 1;
    ma.markers[i].color.r = 0.0f;
    ma.markers[i].color.g = 1.0f;
    ma.markers[i].color.b = 0.0f;
    ma.markers[i].color.a = 1.0f;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // 点の数
    int point_num = scan->scan_time / scan->time_increment;

    static int effect_point_num = 0;      // 有効な点の数
    bool is_passing = false;              // 物体が計測面を通過中か
    static bool was_passing = false;      // 前回のスキャンで物体が計測面を通過したか
    static float buf_pass_point[2] = {0}; // 通過点の座標を成分ごとに積算しておくバッファ

    // すべての点に処理
    for (int i = 0; i < point_num; i++)
    {
        // その点の角度 [rad]
        float angle = scan->angle_min + scan->angle_increment * i;
        // 距離 [m]
        float r = scan->ranges[i];

        // 測定エラー
        if (r == 0)
            continue;

        // 点の座標
        float now_x = r * cos(angle);
        float now_y = r * sin(angle);

        // 計測範囲外
        if (now_x < measur_range[0][0] || now_x > measur_range[0][1])
            continue;
        if (now_y < measur_range[1][0] || now_y > measur_range[1][1])
            continue;

        // 通過中のフラグをたてる
        is_passing = true;

        // 仮の座標に代入し続ける
        buf_pass_point[0] += now_x;
        buf_pass_point[1] += now_y;

        effect_point_num++;
    }

    // 前回通過して、今回は通過していない（立ち下がり）
    if (!is_passing && was_passing)
    {
        pass_point[pass_num][0] = buf_pass_point[0] / effect_point_num;
        pass_point[pass_num][1] = buf_pass_point[1] / effect_point_num;

        float x = pass_point[pass_num][0];
        float y = pass_point[pass_num][1];

        ROS_INFO("[apm]: %d:  (%f, %f)", pass_num, x, y);

        marker_array.markers.resize(pass_num + 1);
        addMarkerArray(marker_array, pass_num, x, y);

        buf_pass_point[0] = 0;
        buf_pass_point[1] = 0;
        effect_point_num = 0;
        pass_num++;
    }

    was_passing = is_passing;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apm_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    ros::Publisher markers_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

    // 計測面を可視化
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker", 1);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/laser_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = measur_range[0][1] - measur_range[0][0];
    marker.scale.y = measur_range[1][1] - measur_range[1][0];
    marker.scale.z = 0.001;
    marker.pose.position.x = 0;
    marker.pose.position.y = abs(measur_range[1][1]);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.5f;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        marker_pub.publish(marker);
        markers_pub.publish(marker_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}