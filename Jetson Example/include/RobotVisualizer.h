// #ifndef ROBOT_VISUALIZER_H
// #define ROBOT_VISUALIZER_H

// #include <open3d/Open3D.h>
// #include <open3d/visualization/gui/Application.h>
// #include <open3d/visualization/gui/Window.h>
// #include <open3d/visualization/gui/SceneWidget.h>
// #include <open3d/visualization/gui/Button.h>
// #include <open3d/visualization/gui/Label.h>
// #include <open3d/visualization/gui/Layout.h>
// #include <open3d/visualization/rendering/Scene.h>
// #include <open3d/visualization/rendering/Material.h>
// #include <opencv2/opencv.hpp>

// #include <memory>
// #include <mutex>
// #include <string>
// #include <atomic>

// class RobotVisualizer {
// public:
//     enum class ViewMode { POINT_CLOUD, VIDEO };
    
//     RobotVisualizer();
//     ~RobotVisualizer();
    
//     // Prevent copying
//     RobotVisualizer(const RobotVisualizer&) = delete;
//     RobotVisualizer& operator=(const RobotVisualizer&) = delete;
    
//     // Initialize and run application
//     bool Initialize();
//     void Run();
    
//     // Update content
//     void UpdatePointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud);
//     void UpdateVideoFrame(const cv::Mat& color_image);
    
//     // Toggle view mode
//     void ToggleView();
//     ViewMode GetCurrentViewMode() const { return view_mode_; }
    
//     // Check if the application is still running
//     bool IsRunning() const { return running_; }

// private:
//     // GUI application and window
//     std::shared_ptr<open3d::visualization::gui::Application> app_;
//     std::shared_ptr<open3d::visualization::gui::Window> window_;
    
//     // GUI widgets
//     std::shared_ptr<open3d::visualization::gui::SceneWidget> scene_widget_;
//     std::shared_ptr<open3d::visualization::gui::Button> toggle_button_;
//     std::shared_ptr<open3d::visualization::gui::Label> status_label_;
    
//     // Scene and materials
//     open3d::visualization::rendering::Scene* scene_ = nullptr;
//     open3d::visualization::rendering::Material point_cloud_material_;
//     open3d::visualization::rendering::Material image_material_;
    
//     // Current data
//     std::shared_ptr<open3d::geometry::PointCloud> current_point_cloud_;
//     std::shared_ptr<open3d::geometry::Image> current_image_;
    
//     // State variables
//     ViewMode view_mode_ = ViewMode::POINT_CLOUD;
//     std::atomic<bool> running_{false};
//     std::atomic<bool> content_updated_{false};
//     std::atomic<bool> point_cloud_updated_{false};
//     std::atomic<bool> video_updated_{false};
    
//     // Thread safety
//     std::mutex data_mutex_;
    
//     // Internal methods
//     void UpdateScene();
//     void CreateImageQuad();
//     std::shared_ptr<open3d::geometry::TriangleMesh> CreateQuadMesh(float width, float height);
// };

// #endif // POINT_CLOUD_VISUALIZER_H