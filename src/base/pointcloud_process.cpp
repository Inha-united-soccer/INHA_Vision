#include "booster_vision/base/pointcloud_process.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace booster_vision {

void VisualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Add the point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

    // Set the background color (optional)
    viewer.setBackgroundColor(0.0, 0.0, 0.0); // Black background

    // Set point size (optional)
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // Add coordinate axis (optional)
    viewer.addCoordinateSystem(1.0);

    // Start the visualization loop
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100); // Process events and update the view
    }
}

void VisualizePointCloudandPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<float> &coeffs, float x, float y, float z) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Add the point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

    // Set the background color (optional)
    viewer.setBackgroundColor(0.0, 0.0, 0.0); // Black background

    // Set point size (optional)
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // Add coordinate axis (optional)
    viewer.addCoordinateSystem(1.0);

    pcl::ModelCoefficients coefficients;
    coefficients.values = coeffs;
    // viewer.addPlane(coefficients, "plane");

    viewer.addPlane(coefficients, x, y, z); // 1 = cumprimento do eixo

    // Start the visualization loop
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100); // Process events and update the view
    }
}

void VisualizePointCloudSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std::vector<std::vector<float>> &sphere_coeffs) {
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Add the point cloud to the visualizer
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

    // Set the background color (optional)
    viewer.setBackgroundColor(0.0, 0.0, 0.0); // Black background

    // Set point size (optional)
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // todo: generate random colors
    std::vector<std::vector<float>> colors{{1, 0, 0}, {0.5, 0.5, 0}, {0, 0.5, 0.5}};
    for (int i = 0; i < sphere_coeffs.size(); i++) {
        const auto &sphere = sphere_coeffs[i];
        pcl::PointXYZRGB center;
        center.x = sphere[0];
        center.y = sphere[1];
        center.z = sphere[2];
        std::string name = "sphere" + std::to_string(i);
        std::vector<float> color = colors[i];
        viewer.addSphere(center, sphere[3], name.c_str());
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2], name.c_str()); // Red color
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name.c_str());
    }

    // Add coordinate axis (optional)
    viewer.addCoordinateSystem(1.0);

    // Start the visualization loop
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100); // Process events and update the view
    }
}

void CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &depth_image, const cv::Mat &rgb_image, const cv::Rect &bbox,
                      const booster_vision::Intrinsics &intrinsics) {
    for (int v = bbox.y; v < bbox.y + bbox.height; ++v) {
        for (int u = bbox.x; u < bbox.x + bbox.width; ++u) {
            float depth = depth_image.at<float>(v, u); // mat.at<T>(row, col)이므로 v먼저 
            if (!std::isnan(depth) && depth > 0) {
                auto cv_point = intrinsics.BackProject(cv::Point2f(u, v), depth);

                pcl::PointXYZRGB point;
                point.x = cv_point.x;
                point.y = cv_point.y;
                point.z = cv_point.z;
                point.b = rgb_image.at<cv::Vec3b>(v, u)[0];
                point.g = rgb_image.at<cv::Vec3b>(v, u)[1];
                point.r = rgb_image.at<cv::Vec3b>(v, u)[2];
                cloud->points.push_back(point);
            }
        }
    }
}

// bbox가 없을 때, 이미지 전체에서 포인트 클라우드 생성하는 함수
void CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &depth_image, const cv::Mat &rgb_image,
                      const booster_vision::Intrinsics &intrinsics) {
    cv::Rect bbox(0, 0, depth_image.cols, depth_image.rows);
    CreatePointCloud(cloud, depth_image, rgb_image, bbox, intrinsics);
}

void DownSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &processed_cloud, const float leaf_size,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // 현재는 1cm 격자로 다운샘플링
    voxel_filter.filter(*processed_cloud);
}

void PointCloudNoiseRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &processed_cloud, const int neighbour_count,
                            const float multiplier,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(neighbour_count);      // Number of nearest neighbors to use
    sor.setStddevMulThresh(multiplier); // Standard deviation multiplier
    sor.filter(*processed_cloud);
}

void ClusterPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_clouds, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       const float cluster_distance_threshold) {
    // 빠른 최근접 이웃 탐색을 위한 KD-트리 객체 생성 -> O(log n) 시간에 최근접 이웃 탐색 가능
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec; // 3D 기반 BFS 클러스터링 객체 생성
    ec.setInputCloud(cloud);
    ec.setClusterTolerance(cluster_distance_threshold); // Adjust tolerance as needed
    ec.setMinClusterSize(100);                          // 최소 100점 이상이어야 클러스터로 인정
    ec.setMaxClusterSize(125000);                       // 125000점 이상이면 클러스터로 인정하지 않음 (너무 큰 클러스터는 노이즈일 가능성 높음)
    ec.setSearchMethod(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    // cluster_indices = [
    //   { indices: [0,5,10,33,...] },
    //   { indices: [3,7,12,...] },
    //   ...
    // ]
    ec.extract(cluster_indices); // 클러스터링 수행, 결과는 cluster_indices에 저장

    std::cout << "number of cluster: " << cluster_indices.size() << std::endl;
    // If no clusters found, return false
    if (cluster_indices.empty()) {
        clustered_clouds = {};
        return;
    }

    for (const auto &indices : cluster_indices) {
        std::cout << "cluster size: " << indices.indices.size() << std::endl;
        // if (indices.indices.size() < cloud->points.size() * 0.20) {
        //   std::cout << "cluster size: " << indices.indices.size() << " smaller than threshold, skip!" << std::endl;
        //   continue; // skip cluster with size smaller than fifth of the input cloud
        // }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int index : indices.indices) {
            cluster->points.push_back(cloud->points[index]);
        }

        clustered_clouds.push_back(cluster);
    }
}

void SphereFitting(std::vector<float> &sphere, float &confidence, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   const float &dist_threshold, const float &radius_threshold) {
    sphere = {0, 0, 0, 0};
    confidence = 0;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;

    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_threshold); // Adjust distance threshold as needed

    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    confidence = static_cast<float>(inliers.indices.size()) / cloud->points.size();
    if (confidence < 0.5) {
        std::cout  << "confidence " << confidence << " lower than expected 0.5, inliner conunts: " << inliers.indices.size() << std::endl;
        return;
    }

    sphere = coefficients.values;
    if (std::abs(coefficients.values[3] - radius_threshold) > 0.02) {
        std::cout << "raidus " << coefficients.values[3] << " higher than expected " << radius_threshold << std::endl;
        confidence = 0;
        return;
    }
    std::cout << " radius: " << coefficients.values[3] << std::endl;
}

void PlaneFitting(std::vector<float> &plane, float &confidence,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const float &dist_threshold) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 평면 방정식 저장
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); // 평면에 속하는 점들의 인덱스
    // RANSAC 세그멘테이션 객체 생성
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    // RANSAC 이후 inlier로 LSE 최적화 수행 여부
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE); // 평면 모델 선택 
    seg.setMethodType(pcl::SAC_RANSAC); // RANSAC 알고리즘 사용
    seg.setDistanceThreshold(dist_threshold); // 평면에서 점까지의 최대 거리, 이보다 멀면 inlier로 간주하지 않음 -> dist_threshold는 노이즈 수준에 따라 조절 필요 (현재는 1cm)

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    plane = {0, 0, 0, 0};
    confidence = 0;
    if (inliers->indices.size() < 100) { // 평면에 속하는 점들의 인덱스
        return;
    }

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    plane = coefficients->values; // 평면 방정식 계수
    confidence = static_cast<float>(inliers->indices.size()) / cloud->size(); // 전체 점 중 몇 퍼센트가 평면에 속하는지 -> 평면 모델의 신뢰도 판단 기준
    std::cout << "inlier percentage: " << confidence << std::endl;

    auto point = cloud->points[inliers->indices[0]];
    // VisualizePointCloudandPlane(cloud, plane, point.x, point.y, point.z);
}

} // namespace booster_vision
