// Include ROS related Headers

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.h>

// Include ROS messages

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


// Include Custom Messages

#include <leg_detector_msgs/msg/leg.hpp>
#include <leg_detector_msgs/msg/leg_array.hpp>

// Include Local Headers

#include <leg_detector/laser_processor.h>

// local constants definition

#define ALPHA 0.2
#define BETA 0.1
#define OBSTACLE 0.7
#define FREE_SPACE 0.4
#define UNKNOWN 0.5
#define MIN_PROB 0.001
#define MAX_PROB 1-MIN_PROB


  using namespace message_filters;

/**
* @basic A simple 'local' occupancy grid map that maps everything except tracked humans
*
* Maps a small area around the robot. The occupied areas on the map are all non-human obstacles.
*/
class OccupancyGridMapping :public rclcpp::Node
{
    public:

        OccupancyGridMapping(): Node("OccupancyGridMapping"), 
                                scan_sub_(this, std::string("scan")), 
                                non_leg_clusters_sub_(this, "non_leg_clusters"),
                                sync(scan_sub_, non_leg_clusters_sub_, 100)
        {
            std::string local_map_topic;
            std::string scan_topic;
            grid_centre_pos_found_ = false;
            
            this->declare_parameter("scan_topic");
            this->declare_parameter("fixed_frame");
            this->declare_parameter("base_frame");
            this->declare_parameter("local_map_topic");
            this->declare_parameter("local_map_resolution");
            this->declare_parameter("local_map_cells_per_side");
            this->declare_parameter("invalid_measurements_are_free_space");
            this->declare_parameter("unseen_is_free_space");
            this->declare_parameter("use_scan_header_stamp_for_tfs");
            this->declare_parameter("shift_threshold");
            this->declare_parameter("reliable_inf_range");
            this->declare_parameter("cluster_dist_euclid");
            this->declare_parameter("min_points_per_cluster");
            
            this->get_parameter_or("scan_topic", scan_topic, std::string("/scan"));
            this->get_parameter_or("fixed_frame", fixed_frame_, std::string("laser"));
            this->get_parameter_or("base_frame", base_frame_, std::string("base_link"));
            this->get_parameter_or("local_map_topic", local_map_topic, std::string("local_map"));
            this->get_parameter_or("local_map_resolution", resolution_, 0.05);
            this->get_parameter_or("local_map_cells_per_side", width_, 400);
            this->get_parameter_or("invalid_measurements_are_free_space", invalid_measurements_are_free_space_, false);
            this->get_parameter_or("unseen_is_freespace", unseen_is_freespace_, true);
            this->get_parameter_or("use_scan_header_stamp_for_tfs", use_scan_header_stamp_for_tfs_, false);

            this->get_parameter_or("shift_threshold", shift_threshold_, 1.0);
            this->get_parameter_or("reliable_inf_range", reliable_inf_range_, 5.0);
            this->get_parameter_or("cluster_dist_euclid", cluster_dist_euclid_, 0.13);
            this->get_parameter_or("min_points_per_cluster", min_points_per_cluster_, 3);

            // Initialize map
            // All probabilities are held in log-space
            l0_ = logit(UNKNOWN);
            l_min_ = logit(MIN_PROB);
            l_max_ = logit(MAX_PROB);
            l_.resize(width_*width_);
            for (int i = 0; i < width_; i++)
            {
                for (int j = 0; j < width_; j++)
                {
                    if (unseen_is_freespace_)
                        l_[i + width_*j] = l_min_;
                    else
                        l_[i + width_*j] = l0_;
                }
            }

            buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
            auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                                                            this->get_node_base_interface(),
                                                            this->get_node_timers_interface());
            buffer_->setCreateTimerInterface(timer_interface);

            scan_sub_.subscribe(this, scan_topic);
            

            // To coordinate callback for both laser scan message and a non_leg_clusters message
            sync.registerCallback(std::bind(&OccupancyGridMapping::laserAndLegCallback, this, std::placeholders::_1, std::placeholders::_2));

            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(local_map_topic, 10);
            markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 20);

        }


    private:
        std::string scan_topic_;
        std::string fixed_frame_;
        std::string base_frame_;

        //-----
        message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
        message_filters::Subscriber<leg_detector_msgs::msg::LegArray> non_leg_clusters_sub_;
        message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, leg_detector_msgs::msg::LegArray> sync;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_pub_;

        //-----

        double l0_;
        std::vector<double> l_;
        double l_min_;
        double l_max_;

        double resolution_;
        int width_;

        bool grid_centre_pos_found_;
        double grid_centre_pos_x_;
        double grid_centre_pos_y_;
        double shift_threshold_;

        rclcpp::Time last_time_;
        bool invalid_measurements_are_free_space_;
        double reliable_inf_range_;
        bool use_scan_header_stamp_for_tfs_;
        rclcpp::Time latest_scan_header_stamp_with_tf_available_;
        bool unseen_is_freespace_;

        double cluster_dist_euclid_;
        int min_points_per_cluster_;

        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;


        /**
         * @brief Coordinated callback for both laser scan message and a non_leg_clusters message
         * 
         * Called whenever both topics have been recently published to
        **/
        void laserAndLegCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg, const leg_detector_msgs::msg::LegArray::ConstSharedPtr& non_leg_clusters) 
        {

            // Find out the time that should be used for tfs
            bool transform_available;
            rclcpp::Time tf_time;

            if(use_scan_header_stamp_for_tfs_)
            {
                tf_time = scan_msg->header.stamp;

                try
                {
                    buffer_->canTransform(fixed_frame_, scan_msg->header.frame_id, tf_time, rclcpp::Duration(1.0));
                    transform_available = buffer_->canTransform(fixed_frame_, scan_msg->header.frame_id, tf_time, rclcpp::Duration(1.0));
                } catch(tf2::TransformException &ex) {
                    RCLCPP_INFO(this->get_logger(), "Local Map : No tf available");
                    transform_available = false;
                }
            } else {

                //Use the latest transform available
                tf_time = rclcpp::Time(0);
                transform_available = buffer_->canTransform(fixed_frame_, scan_msg->header.frame_id, tf_time);
            }
            
            if (transform_available) {

                // Next step: find scan beams that correspond to humans/tracked legs so 
                // we can count them as freespace in the grid occupancy map

                // Transform tracked legs back into the laser frame
                std::vector<geometry_msgs::msg::Point> non_legs;

                for(long unsigned int i = 0; i < non_leg_clusters->legs.size(); i++){

                    leg_detector_msgs::msg::Leg leg = non_leg_clusters->legs[i];

                    geometry_msgs::msg::Point p;
                    p.x = leg.position.x;
                    p.y = leg.position.y;
                    p.z = 0.0;

                    geometry_msgs::msg::PointStamped ps;
                    ps.header.frame_id = fixed_frame_;
                    ps.header.stamp = tf_time;
                    ps.point = p;
                    
                    try {
                        buffer_->transform(ps, scan_msg->header.frame_id, tf2::durationFromSec(0.0));
                        geometry_msgs::msg::Point temp_p = ps.point;
                        non_legs.push_back(temp_p);
                    } catch (tf2::TransformException &ex) {

                        RCLCPP_ERROR(this->get_logger(),"Local map tf error: %s", ex.what());
                    }
                }

                // Determine which scan samples correspond to humans
                // so we can mark those areas as unoccupied in the map
                std::vector<bool> is_sample_human;
                is_sample_human.resize(scan_msg->ranges.size(), false);
                sensor_msgs::msg::LaserScan scan = *scan_msg;
                laser_processor::ScanProcessor processor(scan);
                processor.splitConnected(cluster_dist_euclid_);
                processor.removeLessThan(min_points_per_cluster_);
                for (std::list<laser_processor::SampleSet*>::iterator c_iter = processor.getClusters().begin(); c_iter != processor.getClusters().end(); ++c_iter)
                {
                    bool is_cluster_human = true;
                    geometry_msgs::msg::Point c_pos = (*c_iter)->getPosition();

                    // Check every point in the <non_legs> message to see
                    // if the scan cluster is within an epsilon distance of the cluster
                    for (std::vector<geometry_msgs::msg::Point>::iterator non_leg = non_legs.begin();
                         non_leg != non_legs.end(); ++non_leg) {

                        double dist = sqrt(pow((c_pos.x - (*non_leg).x), 2) + pow((c_pos.y - (*non_leg).y), 2));
                        if (dist < 0.1) {
                            non_legs.erase(non_leg);
                            is_cluster_human = false;
                            break;
                        }
                    }

                    // Set all scan samples in the cluster to <is_cluster_human>
                    for (laser_processor::SampleSet::iterator s_iter = (*c_iter)->begin();
                        s_iter != (*c_iter)->end();
                        ++s_iter) {

                        is_sample_human[(*s_iter)->index] = is_cluster_human;        
                    }
                }

                // Next step: Update the local grid occupancy map
                
                // Get the pose of the laser in the fixed frame
                bool transform_succesful;
                geometry_msgs::msg::PoseStamped init_pose;
                geometry_msgs::msg::PoseStamped laser_pose_fixed_frame;
                init_pose.header.frame_id = scan.header.frame_id;
                init_pose.pose.orientation = createQuaternionMsgFromYaw(0.0);
                init_pose.header.stamp = tf_time;
                
                try {

                    laser_pose_fixed_frame = buffer_->transform(init_pose, fixed_frame_, tf2::durationFromSec(0.0));
                    transform_succesful = true;
                } catch (tf2::TransformException &ex) {
                    RCLCPP_ERROR (this->get_logger(), "Local map tf error: %s", ex.what());
                    transform_succesful = false;
                }

                if(transform_succesful) {

                    // Get the position of the laser
                    double laser_x = laser_pose_fixed_frame.pose.position.x;
                    double laser_y = laser_pose_fixed_frame.pose.position.y;
                    double laser_yaw = tf2::getYaw(laser_pose_fixed_frame.pose.orientation);

                    // Get position of the local occupancy grid relative to the fixed frame
                    if(grid_centre_pos_found_ == false) {
                        grid_centre_pos_found_ = true;
                        grid_centre_pos_x_ = laser_x;
                        grid_centre_pos_y_ = laser_y;
                    }

                    // Check if we need to shift the local grid to be more centred on the laser
                    if (sqrt(pow(grid_centre_pos_x_ - laser_x, 2) + pow(grid_centre_pos_y_ - laser_y, 2)) > shift_threshold_) {

                        // Shifting the local grid
                        int translate_x = -(int)round((grid_centre_pos_x_ - laser_x)/resolution_);
                        int translate_y = -(int)round((grid_centre_pos_y_ - laser_y)/resolution_);

                        // Could translate in place to optimize later if needed
                        std::vector<double> l_translated;
                        l_translated.resize(width_*width_);
                        for (int i = 0; i < width_; i++) {
                            for (int j = 0; j < width_; j++) {
                                int translated_i = i + translate_x;
                                int translated_j = j + translate_y;
                                if (translated_i >= 0 and translated_i < width_ and translated_j >= 0 and translated_j < width_) {
                                    l_translated[i + width_*j] = l_[translated_i + width_*translated_j];
                                } else {
                                    if (unseen_is_freespace_)
                                        l_translated[i + width_*j] = l_min_;
                                    else
                                        l_translated[i + width_*j] = l0_;    
                                }   
                            }
                        }
                        l_ = l_translated;
                        grid_centre_pos_x_ = laser_x;
                        grid_centre_pos_y_ = laser_y;
                    }

                    // Update the local occupancy grid with the new scan
                    for (int i = 0; i < width_; i++) {
                        for (int j = 0; j < width_; j++) {
                            double m_update;

                            // Find dist and angle of current cell to laser position
                            double dist = sqrt(pow(i*resolution_ + grid_centre_pos_x_ - (width_/2.0)*resolution_ - laser_x, 2.0) + pow(j*resolution_ + grid_centre_pos_y_ - (width_/2.0)*resolution_ - laser_y, 2.0));
                            double angle = betweenPIandNegPI(atan2(j*resolution_ + grid_centre_pos_y_ - (width_/2.0)*resolution_ - laser_y, i*resolution_ + grid_centre_pos_x_ - (width_/2.0)*resolution_ - laser_x) - laser_yaw);
                            bool is_human;

                            if (angle > scan.angle_min - scan.angle_increment/2.0 and angle < scan.angle_max + scan.angle_increment/2.0){

                                // Find applicable laser measurement
                                double closest_beam_angle = round(angle/scan.angle_increment)*scan.angle_increment;
                                int idx_without_bounds_check = round(angle/scan.angle_increment) + scan.ranges.size()/2;
                                int closest_beam_idx = std::max(0, std::min(static_cast<int>(scan.ranges.size() - 1), idx_without_bounds_check));
                                is_human = is_sample_human[closest_beam_idx];

                                // Processing the range value of the closest_beam to determine if it's a valid measurement or not.
                                // Sometimes it returns infs and NaNs that have to be dealt with
                                bool valid_measurement;
                                if(scan.range_min <= scan.ranges[closest_beam_idx] && scan.ranges[closest_beam_idx] <= scan.range_max){

                                    // This is a valid measurement.
                                    valid_measurement = true;
                                }
                                else if( !std::isfinite(scan.ranges[closest_beam_idx]) && scan.ranges[closest_beam_idx] < 0){

                                    // Object too close to measure.
                                    valid_measurement = false;
                                }
                                else if( !std::isfinite(scan.ranges[closest_beam_idx] ) && scan.ranges[closest_beam_idx] > 0){

                                    // No objects detected in range.
                                    valid_measurement = true;
                                }
                                else if( std::isnan(scan.ranges[closest_beam_idx]) ){

                                    // This is an erroneous, invalid, or missing measurement.
                                    valid_measurement = false;
                                }
                                else {
                                    // The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
                                    valid_measurement = false;
                                }

                                if (valid_measurement) {
                                    double dist_rel = dist - scan.ranges[closest_beam_idx];
                                    double angle_rel = angle - closest_beam_angle;
                                    if (dist > scan.range_max 
                                        or dist > scan.ranges[closest_beam_idx] + ALPHA/2.0 
                                        or fabs(angle_rel)>BETA/2 
                                        or (!std::isfinite(scan.ranges[closest_beam_idx]) 
                                        and dist > reliable_inf_range_)) {
                                            m_update = UNKNOWN;
                                    }
                                    else if (scan.ranges[closest_beam_idx] < scan.range_max and fabs(dist_rel)<ALPHA/2 and !is_human) {
                                        m_update = OBSTACLE;
                                    }
                                    else {
                                        m_update = FREE_SPACE;
                                    }
                                }
                                else {
                                    // Assume cells corresponding to erroneous measurements are either in freespace or unknown
                                    if (invalid_measurements_are_free_space_)
                                        m_update = FREE_SPACE;
                                    else
                                        m_update = UNKNOWN;   
                                }
                            }
                            else {
                                m_update = UNKNOWN;
                            }

                            // update l_ using m_update
                            l_[i + width_*j] = (l_[i + width_*j] + logit(m_update) - l0_);
                            if (l_[i + width_*j] < l_min_)
                                l_[i + width_*j] = l_min_;
                            else if (l_[i + width_*j] > l_max_)
                                l_[i + width_*j] = l_max_;
                        }
                    }
                    // Create and fill out an OccupancyGrid message
                    nav_msgs::msg::OccupancyGrid m_msg;
                    m_msg.header.stamp = scan_msg->header.stamp; //ros::Time::now();
                    m_msg.header.frame_id = fixed_frame_;
                    m_msg.info.resolution = resolution_;
                    m_msg.info.width = width_;
                    m_msg.info.height = width_;
                    m_msg.info.origin.position.x = grid_centre_pos_x_ - (width_/2.0)*resolution_;
                    m_msg.info.origin.position.y = grid_centre_pos_y_ - (width_/2.0)*resolution_;
                    for (int i = 0; i < width_; i++)
                        for (int j = 0; j < width_; j++)
                            m_msg.data.push_back((int)(inverseLogit(l_[width_*i + j])*100));
                    // Publish!
                    map_pub_->publish(m_msg);
                }
            }
        }

        /**
          * @basic The logit function, i.e., the inverse of the logstic function
          * @param p 
          * @return The logit of p
        **/
        double logit(double p)
        {
            return log(p/(1-p));
        }

        /**
          * @basic The inverse of the logit function, i.e., the logsitic function
          * @param p 
          * @return The inverse logit of p
        **/
        double inverseLogit(double p)
        {
            return exp(p)/(1+exp(p));
        }

        /**
          * @basic Returns the equivilant of a passed-in angle in the -PI to PI range
          * @param angle_in The input angle
          * @return The angle in the range -PI to PI
        **/
        double betweenPIandNegPI(double angle_in)
        {
            double between_0_and_2PI = fmod(angle_in, 2*M_PI);
            if (between_0_and_2PI < M_PI)
            return between_0_and_2PI;
            else
            return between_0_and_2PI - 2*M_PI;
        }

        geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            return tf2::toMsg(q);
        }



};

int main (int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OccupancyGridMapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}