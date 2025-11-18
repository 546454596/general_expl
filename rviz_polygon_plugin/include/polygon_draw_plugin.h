#ifndef POLYGON_DRAW_PLUGIN_H
#define POLYGON_DRAW_PLUGIN_H

#ifndef Q_MOC_RUN
#include <rviz/tool.h>
#include <rviz/panel.h>
#include <rviz/display.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/frame_manager.h>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#endif

#include <QObject>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

namespace polygon_draw_plugin
{

class PolygonDrawTool : public rviz::Tool
{
Q_OBJECT

public:
    PolygonDrawTool();
    virtual ~PolygonDrawTool();

    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();

    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

    void addPoint(const Ogre::Vector3& position);
    void removeLastPoint();
    void clearAllPoints();
    void publishPolygon();

private Q_SLOTS:
    void updateVisualization();

private:
    std::vector<Ogre::Vector3> polygon_points_;
    std::vector<rviz::Line*> lines_;
    std::vector<rviz::Arrow*> point_markers_;
    
    Ogre::SceneNode* scene_node_;
    
    ros::Publisher polygon_pub_;
    ros::Publisher marker_pub_;
    
    std::string frame_id_;
};

class PolygonDrawPanel : public rviz::Panel
{
Q_OBJECT

public:
    PolygonDrawPanel(QWidget* parent = 0);
    virtual ~PolygonDrawPanel();

    virtual void onInitialize();

private Q_SLOTS:
    void onSavePolygon();
    void onClearLastPoint();
    void onClearAllPoints();
    void onPublishPolygon();
    void onTopicChanged();

private:
    QPushButton* save_button_;
    QPushButton* clear_last_button_;
    QPushButton* clear_all_button_;
    QPushButton* publish_button_;
    QLineEdit* topic_edit_;
    QLabel* point_count_label_;
    
    ros::Publisher polygon_pub_;
    ros::Publisher marker_pub_;
    
    PolygonDrawTool* draw_tool_;
    
    void updatePointCount();
};

class PolygonDrawDisplay : public rviz::Display
{
Q_OBJECT

public:
    PolygonDrawDisplay();
    virtual ~PolygonDrawDisplay();

protected:
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);

private Q_SLOTS:
    void updateProperties();

private:
    rviz::StringProperty* topic_property_;
    rviz::ColorProperty* line_color_property_;
    rviz::ColorProperty* point_color_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::FloatProperty* point_size_property_;
    rviz::BoolProperty* show_points_property_;
    
    ros::Subscriber polygon_sub_;
    
    std::vector<rviz::Line*> lines_;
    std::vector<rviz::Arrow*> point_markers_;
    
    Ogre::SceneNode* scene_node_;
    
    void processPolygonMessage(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    void clearVisualization();
    void updateVisualization(const geometry_msgs::PolygonStamped& polygon);
};

} // namespace polygon_draw_plugin

#endif // POLYGON_DRAW_PLUGIN_H