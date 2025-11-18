#include "polygon_draw_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/geometry.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/selection/selection_manager.h>

#include <OgreRay.h>
#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgrePlane.h>

namespace polygon_draw_plugin
{

// PolygonDrawTool Implementation
PolygonDrawTool::PolygonDrawTool()
    : scene_node_(nullptr)
{
    shortcut_key_ = 'p';
}

PolygonDrawTool::~PolygonDrawTool()
{
    clearAllPoints();
}

void PolygonDrawTool::onInitialize()
{
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    
    ros::NodeHandle nh;
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/grid_cover_planner_node/polygon_draw", 1);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/grid_cover_planner_node/polygon_markers", 1);
    
    frame_id_ = "world";
}

void PolygonDrawTool::activate()
{
    ROS_INFO("Polygon Draw Tool activated. Click to add points, right-click to finish.");
}

void PolygonDrawTool::deactivate()
{
    ROS_INFO("Polygon Draw Tool deactivated.");
}

int PolygonDrawTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    if (event.leftDown())
    {
        Ogre::Vector3 intersection;
        Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
        
        if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                            ground_plane,
                                            event.x,
                                            event.y,
                                            intersection))
        {
            addPoint(intersection);
            return Render;
        }
    }
    else if (event.rightDown())
    {
        if (polygon_points_.size() >= 3)
        {
            publishPolygon();
        }
        else
        {
            ROS_WARN("Need at least 3 points to create a polygon!");
        }
        return Render;
    }
    
    return 0;
}

void PolygonDrawTool::addPoint(const Ogre::Vector3& position)
{
    polygon_points_.push_back(position);
    
    // Create point marker
    rviz::Arrow* point_marker = new rviz::Arrow(scene_manager_, scene_node_);
    point_marker->setPosition(position);
    point_marker->setOrientation(Ogre::Quaternion::IDENTITY);
    point_marker->setScale(Ogre::Vector3(0.2f, 0.2f, 0.2f));
    point_marker->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    point_markers_.push_back(point_marker);
    
    // Create line to previous point
    if (polygon_points_.size() > 1)
    {
        rviz::Line* line = new rviz::Line(scene_manager_, scene_node_);
        line->setPoints(polygon_points_[polygon_points_.size() - 2], position);
        line->setColor(1.0f, 0.0f, 0.0f, 1.0f);
        lines_.push_back(line);
    }
    
    ROS_INFO("Added point %zu: (%.2f, %.2f, %.2f)", 
             polygon_points_.size(), position.x, position.y, position.z);
}

void PolygonDrawTool::removeLastPoint()
{
    if (polygon_points_.empty()) return;
    
    // Remove last point marker
    if (!point_markers_.empty())
    {
        delete point_markers_.back();
        point_markers_.pop_back();
    }
    
    // Remove last line
    if (!lines_.empty())
    {
        delete lines_.back();
        lines_.pop_back();
    }
    
    polygon_points_.pop_back();
    
    ROS_INFO("Removed last point. Current points: %zu", polygon_points_.size());
}

void PolygonDrawTool::clearAllPoints()
{
    // Clear all lines
    for (auto line : lines_)
    {
        delete line;
    }
    lines_.clear();
    
    // Clear all point markers
    for (auto marker : point_markers_)
    {
        delete marker;
    }
    point_markers_.clear();
    
    polygon_points_.clear();
    
    ROS_INFO("Cleared all points");
}

void PolygonDrawTool::publishPolygon()
{
    if (polygon_points_.size() < 3)
    {
        ROS_WARN("Cannot publish polygon with less than 3 points!");
        return;
    }
    
    // Publish PolygonStamped
    geometry_msgs::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = ros::Time::now();
    polygon_msg.header.frame_id = frame_id_;
    
    for (const auto& point : polygon_points_)
    {
        geometry_msgs::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        polygon_msg.polygon.points.push_back(p);
    }
    ROS_WARN("Publish polygon msg from plugin");
    polygon_pub_.publish(polygon_msg);
    
    // Publish visualization markers
    visualization_msgs::MarkerArray marker_array;
    
    // Line strip marker for polygon edges
    visualization_msgs::Marker line_marker;
    line_marker.header = polygon_msg.header;
    line_marker.ns = "polygon_edges";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.scale.x = 0.1;
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    
    for (const auto& point : polygon_points_)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        line_marker.points.push_back(p);
    }
    
    // Close the polygon
    if (!polygon_points_.empty())
    {
        geometry_msgs::Point p;
        p.x = polygon_points_[0].x;
        p.y = polygon_points_[0].y;
        p.z = polygon_points_[0].z;
        line_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(line_marker);
    
    // Point markers for vertices
    for (size_t i = 0; i < polygon_points_.size(); ++i)
    {
        visualization_msgs::Marker point_marker;
        point_marker.header = polygon_msg.header;
        point_marker.ns = "polygon_vertices";
        point_marker.id = i;
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;
        
        point_marker.pose.position.x = polygon_points_[i].x;
        point_marker.pose.position.y = polygon_points_[i].y;
        point_marker.pose.position.z = polygon_points_[i].z;
        point_marker.pose.orientation.w = 1.0;
        
        point_marker.scale.x = 0.5;
        point_marker.scale.y = 0.5;
        point_marker.scale.z = 0.5;
        
        point_marker.color.r = 0.0;
        point_marker.color.g = 1.0;
        point_marker.color.b = 0.0;
        point_marker.color.a = 1.0;
        
        marker_array.markers.push_back(point_marker);
    }
    
    marker_pub_.publish(marker_array);
    
    ROS_INFO("Published polygon with %zu points", polygon_points_.size());
}

void PolygonDrawTool::updateVisualization()
{
    // This method can be called to update the visualization
    // Currently, visualization is updated immediately when points are added
}

// PolygonDrawPanel Implementation
PolygonDrawPanel::PolygonDrawPanel(QWidget* parent)
    : rviz::Panel(parent)
    , draw_tool_(nullptr)
{
    QVBoxLayout* layout = new QVBoxLayout;
    
    // Topic input
    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget(new QLabel("Topic:"));
    topic_edit_ = new QLineEdit("polygon_draw");
    topic_layout->addWidget(topic_edit_);
    layout->addLayout(topic_layout);
    
    // Point count display
    point_count_label_ = new QLabel("Points: 0");
    layout->addWidget(point_count_label_);
    
    // Buttons
    save_button_ = new QPushButton("Save & Publish Polygon");
    clear_last_button_ = new QPushButton("Remove Last Point");
    clear_all_button_ = new QPushButton("Clear All Points");
    publish_button_ = new QPushButton("Publish Current Polygon");
    
    layout->addWidget(save_button_);
    layout->addWidget(clear_last_button_);
    layout->addWidget(clear_all_button_);
    layout->addWidget(publish_button_);
    
    setLayout(layout);
    
    // Connect signals
    connect(save_button_, SIGNAL(clicked()), this, SLOT(onSavePolygon()));
    connect(clear_last_button_, SIGNAL(clicked()), this, SLOT(onClearLastPoint()));
    connect(clear_all_button_, SIGNAL(clicked()), this, SLOT(onClearAllPoints()));
    connect(publish_button_, SIGNAL(clicked()), this, SLOT(onPublishPolygon()));
    connect(topic_edit_, SIGNAL(textChanged(QString)), this, SLOT(onTopicChanged()));
}

PolygonDrawPanel::~PolygonDrawPanel()
{
}

void PolygonDrawPanel::onInitialize()
{
    ros::NodeHandle nh;
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>(
        topic_edit_->text().toStdString(), 1);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        topic_edit_->text().toStdString() + "_markers", 1);
}

void PolygonDrawPanel::onSavePolygon()
{
    if (draw_tool_)
    {
        draw_tool_->publishPolygon();
        draw_tool_->clearAllPoints();
        updatePointCount();
    }
}

void PolygonDrawPanel::onClearLastPoint()
{
    if (draw_tool_)
    {
        draw_tool_->removeLastPoint();
        updatePointCount();
    }
}

void PolygonDrawPanel::onClearAllPoints()
{
    if (draw_tool_)
    {
        draw_tool_->clearAllPoints();
        updatePointCount();
    }
}

void PolygonDrawPanel::onPublishPolygon()
{
    if (draw_tool_)
    {
        draw_tool_->publishPolygon();
    }
}

void PolygonDrawPanel::onTopicChanged()
{
    ros::NodeHandle nh;
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>(
        topic_edit_->text().toStdString(), 1);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        topic_edit_->text().toStdString() + "_markers", 1);
}

void PolygonDrawPanel::updatePointCount()
{
    if (draw_tool_)
    {
        // This would need to be implemented to get the actual point count
        // For now, we'll show a placeholder
        point_count_label_->setText(QString("Points: %1").arg(0));
    }
}

// PolygonDrawDisplay Implementation
PolygonDrawDisplay::PolygonDrawDisplay()
    : scene_node_(nullptr)
{
    topic_property_ = new rviz::StringProperty("Topic", "polygon_draw",
                                              "Topic to subscribe to for polygon data",
                                              this, SLOT(updateProperties()));
    
    line_color_property_ = new rviz::ColorProperty("Line Color", QColor(255, 0, 0),
                                                  "Color of polygon edges",
                                                  this, SLOT(updateProperties()));
    
    point_color_property_ = new rviz::ColorProperty("Point Color", QColor(0, 255, 0),
                                                   "Color of polygon vertices",
                                                   this, SLOT(updateProperties()));
    
    line_width_property_ = new rviz::FloatProperty("Line Width", 0.1f,
                                                  "Width of polygon edges",
                                                  this, SLOT(updateProperties()));
    
    point_size_property_ = new rviz::FloatProperty("Point Size", 0.2f,
                                                  "Size of polygon vertices",
                                                  this, SLOT(updateProperties()));
    
    show_points_property_ = new rviz::BoolProperty("Show Points", true,
                                                  "Whether to show polygon vertices",
                                                  this, SLOT(updateProperties()));
}

PolygonDrawDisplay::~PolygonDrawDisplay()
{
    clearVisualization();
}

void PolygonDrawDisplay::onInitialize()
{
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    
    ros::NodeHandle nh;
    polygon_sub_ = nh.subscribe<geometry_msgs::PolygonStamped>(
        topic_property_->getStdString(), 1,
        boost::bind(&PolygonDrawDisplay::processPolygonMessage, this, _1));
}

void PolygonDrawDisplay::onEnable()
{
    scene_node_->setVisible(true);
}

void PolygonDrawDisplay::onDisable()
{
    scene_node_->setVisible(false);
}

void PolygonDrawDisplay::update(float wall_dt, float ros_dt)
{
    // Update loop - can be used for animations or dynamic updates
}

void PolygonDrawDisplay::updateProperties()
{
    // Resubscribe if topic changed
    ros::NodeHandle nh;
    polygon_sub_ = nh.subscribe<geometry_msgs::PolygonStamped>(
        topic_property_->getStdString(), 1,
        boost::bind(&PolygonDrawDisplay::processPolygonMessage, this, _1));
}

void PolygonDrawDisplay::processPolygonMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    clearVisualization();
    updateVisualization(*msg);
}

void PolygonDrawDisplay::clearVisualization()
{
    for (auto line : lines_)
    {
        delete line;
    }
    lines_.clear();
    
    for (auto marker : point_markers_)
    {
        delete marker;
    }
    point_markers_.clear();
}

void PolygonDrawDisplay::updateVisualization(const geometry_msgs::PolygonStamped& polygon)
{
    if (polygon.polygon.points.size() < 3) return;
    
    QColor line_color = line_color_property_->getColor();
    QColor point_color = point_color_property_->getColor();
    float line_width = line_width_property_->getFloat();
    float point_size = point_size_property_->getFloat();
    bool show_points = show_points_property_->getBool();
    
    // Create lines for polygon edges
    for (size_t i = 0; i < polygon.polygon.points.size(); ++i)
    {
        size_t next_i = (i + 1) % polygon.polygon.points.size();
        
        Ogre::Vector3 start(polygon.polygon.points[i].x,
                           polygon.polygon.points[i].y,
                           polygon.polygon.points[i].z);
        Ogre::Vector3 end(polygon.polygon.points[next_i].x,
                         polygon.polygon.points[next_i].y,
                         polygon.polygon.points[next_i].z);
        
        rviz::Line* line = new rviz::Line(scene_manager_, scene_node_);
        line->setPoints(start, end);
        line->setColor(line_color.redF(), line_color.greenF(), line_color.blueF(), 1.0f);
        lines_.push_back(line);
    }
    
    // Create point markers for vertices
    if (show_points)
    {
        for (const auto& point : polygon.polygon.points)
        {
            Ogre::Vector3 position(point.x, point.y, point.z);
            
            rviz::Arrow* marker = new rviz::Arrow(scene_manager_, scene_node_);
            marker->setPosition(position);
            marker->setOrientation(Ogre::Quaternion::IDENTITY);
            marker->setScale(Ogre::Vector3(point_size, point_size, point_size));
            marker->setColor(point_color.redF(), point_color.greenF(), point_color.blueF(), 1.0f);
            point_markers_.push_back(marker);
        }
    }
}

} // namespace polygon_draw_plugin

// Register the plugin classes with pluginlib
PLUGINLIB_EXPORT_CLASS(polygon_draw_plugin::PolygonDrawTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(polygon_draw_plugin::PolygonDrawPanel, rviz::Panel)
PLUGINLIB_EXPORT_CLASS(polygon_draw_plugin::PolygonDrawDisplay, rviz::Display)

#include <pluginlib/class_list_macros.h>