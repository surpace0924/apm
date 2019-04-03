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

        // 平均を算出
        for (int i = 0; i < 2; i++)
        {
            float sum = 0;
            for (int j = 0; j <= pass_num; j++)
            {
                sum += pass_point[j][i];
            }
            pass_point_ave[i] = sum / (pass_num + 1);
        }
        // 分散を算出
        for (int i = 0; i < 2; i++)
        {
            float sum = 0;
            for (int j = 0; j <= pass_num; j++)
            {
                sum += (pass_point[j][i] - pass_point_ave[i]) * (pass_point[j][i] - pass_point_ave[i]);
            }
            pass_point_sig[i] = sum / (pass_num + 1);
            pass_point_sig[i] = sqrt(pass_point_sig[i]);
        }

        if (pass_num != 0)
            generateStatisticsSurface(statistics_surface);

        marker_array.markers.resize(pass_num + 1);
        addMarkerArray(marker_array, pass_num, x, y);

        pass_time[pass_num] = nowtime();

        ROS_INFO("%d: %s  (%f, %f)", pass_num, nowtime().c_str(), x, y);

        std::stringstream ss;
        ss << "index: " << pass_num << "\r\n"
           << "通過時間: " << pass_time[pass_num] << "\r\n"
           << "x: " << pass_point[pass_num][0] << "\r\n"
           << "y: " << pass_point[pass_num][1] << "\r\n"
           << "\r\n"
           << "-平均-\r\n"
           << "x: " << pass_point_ave[0] << "\r\n"
           << "y: " << pass_point_ave[1] << "\r\n"
           << "\r\n"
           << "-偏差-\r\n"
           << "x: " << pass_point_sig[0] << "\r\n"
           << "y: " << pass_point_sig[1];
        std::string send_str = ss.str();

        sendText(text, send_str);

        buf_pass_point[0] = 0;
        buf_pass_point[1] = 0;
        effect_point_num = 0;
        pass_num++;
    }

    was_passing = is_passing;
}
