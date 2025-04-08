// #include "RobotVisualizer.h"

// RobotVisualizer::RobotVisualizer() {
// }

// RobotVisualizer::~RobotVisualizer() {
//     running_ = false;
// }

// bool RobotVisualizer::Initialize() {
//     // Initialize Open3D application
//     app_ = std::make_shared<open3d::visualization::gui::Application>();
//     app_->Initialize();
    
//     // Create window
//     window_ = std::make_shared<open3d::visualization::gui::Window>(
//         "Point Cloud & Video Visualizer", 1280, 720);
    
//     // Create scene widget for 3D content
//     scene_widget_ = std::make_shared<open3d::visualization::gui::SceneWidget>();
//     scene_ = scene_widget_->GetScene();
    
//     // Set up material for point cloud
//     point_cloud_material_.shader = "defaultLit";
    
//     // Set up material for image display
//     image_material_.shader = "unlitTexture";
    
//     // Create the toggle button
//     toggle_button_ = std::make_shared<open3d::visualization::gui::Button>("Toggle View (T)");
//     toggle_button_->SetOnClicked([this]() {
//         this->ToggleView();
//     });
    
//     // Create status label
//     status_label_ = std::make_shared<open3d::visualization::gui::Label>("Current View: Point Cloud");
    
//     // Set up bottom toolbar
//     auto toolbar = std::make_shared<open3d::visualization::gui::Horiz>(
//         0, open3d::visualization::gui::Margins(0.5, 0, 0.5, 0));
//     toolbar->AddChild(status_label_);
//     toolbar->AddStretch();
//     toolbar->AddChild(toggle_button_);
    
//     // Set up main layout
//     auto layout = std::make_shared<open3d::visualization::gui::Vert>(
//         0, open3d::visualization::gui::Margins(0, 0, 0, 0));
//     layout->AddChild(scene_widget_);
//     layout->AddFixed(50);  // Fixed height for toolbar
//     layout->AddChild(toolbar);
    
//     window_->AddChild(layout);
    
//     // Register key callback for "T" (toggle)
//     window_->SetOnKeyEvent([this](int key, int action, int mods) -> bool {
//         if (key == 'T' && action == open3d::visualization::gui::KeyAction::KEY_DOWN) {
//             this->ToggleView();
//             return true;
//         }
//         return false;
//     });
    
//     // Register close callback
//     window_->SetOnClose([this]() {
//         this->running_ = false;
//         return true;
//     });
    
//     // Add window to app
//     app_->AddWindow(window_);
//     running_ = true;
    
//     return true;
// }

// void RobotVisualizer::Run() {
//     if (app_) {
//         app_->Run();
//     }
// }

// void RobotVisualizer::UpdatePointCloud(
//     const std::shared_ptr<open3d::geometry::PointCloud>& point_cloud) {
//     if (!running_) return;
    
//     {
//         std::lock_guard<std::mutex> lock(data_mutex_);
//         current_point_cloud_ = point_cloud;
//         point_cloud_updated_ = true;
//         content_updated_ = true;
//     }
    
//     if (view_mode_ == ViewMode::POINT_CLOUD) {
//         // Post a UI update request
//         open3d::visualization::gui::Application::GetInstance().PostToMainThread(
//             window_, [this]() { this->UpdateScene(); });
//     }
// }

// void RobotVisualizer::UpdateVideoFrame(const cv::Mat& color_image) {
//     if (!running_ || color_image.empty()) return;
    
//     {
//         std::lock_guard<std::mutex> lock(data_mutex_);
        
//         // Convert OpenCV image to Open3D image
//         if (!current_image_) {
//             current_image_ = std::make_shared<open3d::geometry::Image>();
//         }
        
//         // Make sure image is BGR or RGB format
//         int channels = color_image.channels();
//         if (channels != 3) {
//             return;  // Unsupported format
//         }
        
//         current_image_->Prepare(
//             color_image.cols, color_image.rows, channels, sizeof(uint8_t));
        
//         // Copy data from OpenCV Mat to Open3D Image
//         std::memcpy(
//             current_image_->data_.data(), 
//             color_image.data, 
//             color_image.total() * color_image.elemSize());
        
//         video_updated_ = true;
//         content_updated_ = true;
//     }
    
//     if (view_mode_ == ViewMode::VIDEO) {
//         // Post a UI update request
//         open3d::visualization::gui::Application::GetInstance().PostToMainThread(
//             window_, [this]() { this->UpdateScene(); });
//     }
// }

// void RobotVisualizer::ToggleView() {
//     if (!running_) return;
    
//     {
//         std::lock_guard<std::mutex> lock(data_mutex_);
//         view_mode_ = (view_mode_ == ViewMode::POINT_CLOUD) ? 
//                      ViewMode::VIDEO : ViewMode::POINT_CLOUD;
                     
//         // Update status label
//         status_label_->SetText(
//             view_mode_ == ViewMode::POINT_CLOUD ? 
//             "Current View: Point Cloud" : "Current View: Video Stream");
//     }
    
//     // Post a UI update request
//     open3d::visualization::gui::Application::GetInstance().PostToMainThread(
//         window_, [this]() { this->UpdateScene(); });
// }

// void RobotVisualizer::UpdateScene() {
//     std::lock_guard<std::mutex> lock(data_mutex_);
    
//     // Clear existing geometry
//     scene_->ClearGeometry();
    
//     if (view_mode_ == ViewMode::POINT_CLOUD) {
//         if (current_point_cloud_ && point_cloud_updated_) {
//             // Add point cloud to scene
//             scene_->AddGeometry(
//                 "point_cloud", current_point_cloud_.get(), point_cloud_material_);
                
//             // Reset camera to focus on point cloud if needed
//             if (point_cloud_updated_) {
//                 Eigen::Vector3d center = current_point_cloud_->GetCenter();
//                 scene_widget_->SetupCamera(60.0, center, 
//                                          center + Eigen::Vector3d(0, 0, 3),
//                                          Eigen::Vector3d(0, -1, 0));
//                 point_cloud_updated_ = false;
//             }
//         }
//     } else if (view_mode_ == ViewMode::VIDEO) {
//         if (current_image_ && video_updated_) {
//             // Create a quad mesh to display the image
//             auto quad_mesh = CreateQuadMesh(4.0, 3.0);  // 4:3 aspect ratio
            
//             // Create a texture from the image
//             auto texture = std::make_shared<open3d::visualization::rendering::Texture>();
//             texture->SetImage(current_image_);
            
//             // Set the texture to the material
//             image_material_.SetTexture("albedo", texture);
            
//             // Add the quad to the scene with the textured material
//             scene_->AddGeometry("video_frame", quad_mesh.get(), image_material_);
            
//             // Set camera to look at the quad
//             scene_widget_->SetupCamera(60.0, 
//                                      Eigen::Vector3d(0, 0, 0),
//                                      Eigen::Vector3d(0, 0, 2),
//                                      Eigen::Vector3d(0, -1, 0));
            
//             video_updated_ = false;
//         }
//     }
    
//     content_updated_ = false;
// }

// std::shared_ptr<open3d::geometry::TriangleMesh> RobotVisualizer::CreateQuadMesh(
//     float width, float height) {
//     auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    
//     float half_w = width / 2.0f;
//     float half_h = height / 2.0f;
    
//     // Add vertices
//     mesh->vertices_.push_back(Eigen::Vector3d(-half_w, -half_h, 0));  // bottom-left
//     mesh->vertices_.push_back(Eigen::Vector3d(half_w, -half_h, 0));   // bottom-right
//     mesh->vertices_.push_back(Eigen::Vector3d(half_w, half_h, 0));    // top-right
//     mesh->vertices_.push_back(Eigen::Vector3d(-half_w, half_h, 0));   // top-left
    
//     // Add triangles (2 triangles for the quad)
//     mesh->triangles_.push_back(Eigen::Vector3i(0, 1, 2));  // bottom-left, bottom-right, top-right
//     mesh->triangles_.push_back(Eigen::Vector3i(0, 2, 3));  // bottom-left, top-right, top-left
    
//     // Add texture coordinates for proper image mapping
//     mesh->triangle_uvs_.push_back(Eigen::Vector2d(0, 1));  // bottom-left
//     mesh->triangle_uvs_.push_back(Eigen::Vector2d(1, 1));  // bottom-right
//     mesh->triangle_uvs_.push_back(Eigen::Vector2d(1, 0));  // top-right
    
//     mesh->triangle_uvs_.push_back(Eigen::Vector2d(0, 1));  // bottom-left
//     mesh->triangle_uvs_.push_back(Eigen::Vector2d(1, 0));  // top-right
//     mesh->triangle_uvs_.push_back(Eigen::Vector2d(0, 0));  // top-left
    
//     return mesh;
// }