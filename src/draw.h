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

void generateMeasurSurface(visualization_msgs::Marker &m)
{
    m.header.frame_id = "/laser_frame";
    m.header.stamp = ros::Time::now();
    m.ns = "basic_shapes";
    m.id = 0;

    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration();

    m.scale.x = measur_range[0][1] - measur_range[0][0];
    m.scale.y = measur_range[1][1] - measur_range[1][0];
    m.scale.z = 0.001;
    m.pose.position.x = 0;
    m.pose.position.y = abs(measur_range[1][1]);
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.color.a = 0.5f;
}

void generateStatisticsSurface(visualization_msgs::Marker &m)
{
    m.header.frame_id = "/laser_frame";
    m.header.stamp = ros::Time::now();
    m.ns = "basic_shapes";
    m.id = 1;

    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration();

    m.scale.x = 2 * 3 * pass_point_sig[0];
    m.scale.y = 2 * 3 * pass_point_sig[1];
    m.scale.z = 0.001;
    m.pose.position.x = pass_point_ave[0];
    m.pose.position.y = pass_point_ave[1];
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.5f;
}