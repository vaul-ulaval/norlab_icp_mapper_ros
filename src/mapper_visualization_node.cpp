#include <memory>
#include <string>
#include <fstream>

#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace PMS = PointMatcherSupport;

class MapperVisualizationNode : public rclcpp::Node
{
public:
    MapperVisualizationNode() : Node("mapper_visualization_node")
    {
        this->declare_parameter<std::string>("input_filters_config", "");
        this->declare_parameter<std::string>("output_filters_config", "");

        std::string scanFiltersConfigFilePath = this->get_parameter("input_filters_config").get_value<std::string>();
        if(!scanFiltersConfigFilePath.empty())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Loading input filters config file " << scanFiltersConfigFilePath);
            std::ifstream ifs(scanFiltersConfigFilePath);
            this->scanFilters = PM::DataPointsFilters(ifs);
            this->scanFilters.init();
            ifs.close();
        }

        std::string outputFiltersConfigFilePath = this->get_parameter("output_filters_config").get_value<std::string>();
        if(!outputFiltersConfigFilePath.empty())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Loading output filters config file " << outputFiltersConfigFilePath);
            std::ifstream ifs(outputFiltersConfigFilePath);
            this->mapSavingFilters = PM::DataPointsFilters(ifs);
            this->mapSavingFilters.init();
            ifs.close();
        }

        rclcpp::QoS mapQoS = rclcpp::SystemDefaultsQoS().keep_last(1).best_effort().transient_local();
        this->mapNormalsSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map",
            mapQoS,
            std::bind(
                &MapperVisualizationNode::mapNormalsVisualization,
                this,
                std::placeholders::_1
            )
        );
        this->mapObservationDirectionsSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map",
            mapQoS,
            std::bind(
                &MapperVisualizationNode::mapObservationDirectionsVisualization,
                this,
                std::placeholders::_1
            )
        );
        this->mapOctreeGridSubscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "map",
            mapQoS,
            std::bind(
                &MapperVisualizationNode::mapOctreeGridVisualization,
                this,
                std::placeholders::_1
            )
        );

        rclcpp::QoS scanQoS = rclcpp::SensorDataQoS();
        this->scanNormalsSubscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "input",
            rclcpp::SensorDataQoS(),
            std::bind(
                &MapperVisualizationNode::scanNormalsVisualization,
                this,
                std::placeholders::_1
            )
        );
        this->scanObservationDirectionsSubscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "input",
            rclcpp::SensorDataQoS(),
            std::bind(
                &MapperVisualizationNode::scanObservationDirectionsVisualization,
                this,
                std::placeholders::_1
            )
        );

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort().transient_local();
        this->normalsVisualizationPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/normals", qos);
        this->observationDirectionVisualizationPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/observation_directions", qos);
        this->OctreeGridVisualizationPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualization/octree_grid", qos);
    }

private:
    using PM = PointMatcher<float>;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapNormalsSubscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapObservationDirectionsSubscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapOctreeGridSubscription;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanNormalsSubscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanObservationDirectionsSubscription;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr normalsVisualizationPublisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr observationDirectionVisualizationPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr OctreeGridVisualizationPublisher;
    PM::DataPointsFilters scanFilters;
    PM::DataPointsFilters mapSavingFilters;

    void mapNormalsVisualization(const sensor_msgs::msg::PointCloud2::SharedPtr mapMsg)
    {
        const PM::DataPoints& mapDataPoints = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(*mapMsg);

        if (mapDataPoints.descriptorExists("normals"))
        {
            visualization_msgs::msg::MarkerArray normalsMarkers;

            for (size_t i = 0; i < mapDataPoints.getNbPoints(); ++i)
            {
                normalsMarkers.markers.emplace_back(constructNormalMarker(mapMsg->header.frame_id, mapMsg->header.stamp, mapDataPoints, i));
            }

            this->normalsVisualizationPublisher->publish(normalsMarkers);
        }
    }

    void mapObservationDirectionsVisualization(const sensor_msgs::msg::PointCloud2::SharedPtr mapMsg)
    {
        const PM::DataPoints& mapDataPoints = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(*mapMsg);

        if (mapDataPoints.descriptorExists("observationDirections"))
        {
            visualization_msgs::msg::MarkerArray observationDirectionsMarkers;

            for (size_t i = 0; i < mapDataPoints.getNbPoints(); ++i)
            {
                observationDirectionsMarkers.markers.emplace_back(constructObservationDirectionMarker(mapMsg->header.frame_id, mapMsg->header.stamp, mapDataPoints, i));
            }

            this->observationDirectionVisualizationPublisher->publish(observationDirectionsMarkers);
        }
    }

    void mapOctreeGridVisualization(const sensor_msgs::msg::PointCloud2::SharedPtr mapMsg)
    {
        PM::DataPoints mapDataPoints = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(*mapMsg);
        this->mapSavingFilters.apply(mapDataPoints);

        sensor_msgs::msg::PointCloud2 mapFilteredMsg = PointMatcher_ROS::pointMatcherCloudToRosMsg<PM::ScalarType>(mapDataPoints, mapMsg->header.frame_id, mapMsg->header.stamp);
        this->OctreeGridVisualizationPublisher->publish(mapFilteredMsg);
    }

    void scanNormalsVisualization(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg)
    {
        PM::DataPoints scanDataPoints = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(*scanMsg);

        this->scanFilters.apply(scanDataPoints);

        if (scanDataPoints.descriptorExists("normals"))
        {
            visualization_msgs::msg::MarkerArray normalsMarkers;

            for (size_t i = 0; i < scanDataPoints.getNbPoints(); ++i)
            {
                normalsMarkers.markers.emplace_back(constructNormalMarker(scanMsg->header.frame_id, scanMsg->header.stamp, scanDataPoints, i));
            }

            this->normalsVisualizationPublisher->publish(normalsMarkers);
        }
    }

    void scanObservationDirectionsVisualization(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg)
    {
        PM::DataPoints scanDataPoints = PointMatcher_ROS::rosMsgToPointMatcherCloud<PM::ScalarType>(*scanMsg);

        this->scanFilters.apply(scanDataPoints);

        if (scanDataPoints.descriptorExists("observationDirections"))
        {
            visualization_msgs::msg::MarkerArray observationDirectionsMarkers;

            for (size_t i = 0; i < scanDataPoints.getNbPoints(); ++i)
            {
                observationDirectionsMarkers.markers.emplace_back(constructObservationDirectionMarker(scanMsg->header.frame_id, scanMsg->header.stamp, scanDataPoints, i));
            }

            this->observationDirectionVisualizationPublisher->publish(observationDirectionsMarkers);
        }
    }

    const visualization_msgs::msg::Marker constructNormalMarker(const std::string& frame_id, const rclcpp::Time& stamp, const PM::DataPoints& map, const size_t index) const
    {
        PM::DataPoints::ConstView normalsView(map.getDescriptorViewByName("normals"));

        visualization_msgs::msg::Marker normalMarker;
        normalMarker.header.frame_id = frame_id;
        normalMarker.header.stamp = stamp;
        normalMarker.id = index;
        normalMarker.ns = frame_id;
        normalMarker.type = visualization_msgs::msg::Marker::ARROW;
        normalMarker.action = visualization_msgs::msg::Marker::ADD;

        normalMarker.pose.position.x = map.features(0, index);
        normalMarker.pose.position.y = map.features(1, index);
        normalMarker.pose.position.z = 0.0f;

        PM::Vector normal = normalsView.col(index);
        const PM::Quaternion normalOrientation = computeOrientationFromVector(normal);

        normalMarker.pose.orientation.x = normalOrientation.x();
        normalMarker.pose.orientation.y = normalOrientation.y();
        normalMarker.pose.orientation.z = normalOrientation.z();
        normalMarker.pose.orientation.w = normalOrientation.w();
        normalMarker.scale.x = 0.1f;
        normalMarker.scale.y = 0.0025f;
        normalMarker.scale.z = 0.0025f;
        normalMarker.color.r = 1.f;
        normalMarker.color.g = 0.f;
        normalMarker.color.b = 1.f;
        normalMarker.color.a = 0.4f;
        normalMarker.lifetime = rclcpp::Duration(0);
        normalMarker.frame_locked = true;

        return normalMarker;
    }

    const visualization_msgs::msg::Marker constructObservationDirectionMarker(const std::string& frame_id, const rclcpp::Time& stamp, const PM::DataPoints& map, size_t index) const
    {
        PM::DataPoints::ConstView observationDirectionsView(map.getDescriptorViewByName("observationDirections"));

        visualization_msgs::msg::Marker observationDirectionMarker;
        observationDirectionMarker.header.frame_id = frame_id;
        observationDirectionMarker.header.stamp = stamp;
        observationDirectionMarker.id = index;
        observationDirectionMarker.ns = frame_id;
        observationDirectionMarker.type = visualization_msgs::msg::Marker::ARROW;
        observationDirectionMarker.action = visualization_msgs::msg::Marker::ADD;

        observationDirectionMarker.pose.position.x = map.features(0, index);
        observationDirectionMarker.pose.position.y = map.features(1, index);
        observationDirectionMarker.pose.position.z = 0.0f;

        PM::Vector observationDirection = observationDirectionsView.col(index);
        const PM::Quaternion observationDirectionOrientation = computeOrientationFromVector(observationDirection);

        observationDirectionMarker.pose.orientation.x = observationDirectionOrientation.x();
        observationDirectionMarker.pose.orientation.y = observationDirectionOrientation.y();
        observationDirectionMarker.pose.orientation.z = observationDirectionOrientation.z();
        observationDirectionMarker.pose.orientation.w = observationDirectionOrientation.w();
        observationDirectionMarker.scale.x = 0.05f;
        observationDirectionMarker.scale.y = 0.0025f;
        observationDirectionMarker.scale.z = 0.0025f;
        observationDirectionMarker.color.r = 0.f;
        observationDirectionMarker.color.g = 1.f;
        observationDirectionMarker.color.b = 1.f;
        observationDirectionMarker.color.a = 0.4f;
        observationDirectionMarker.lifetime = rclcpp::Duration(0);
        observationDirectionMarker.frame_locked = true;

        return observationDirectionMarker;
    }

    const PM::Quaternion computeOrientationFromVector(const PM::Vector& vector) const
    {
        float theta = std::atan2(vector(1), vector(0));
        PM::Quaternion quaternion(Eigen::AngleAxis<PM::ScalarType>(theta, Eigen::Vector3f::UnitZ()));
        quaternion.normalize();
        return quaternion;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
