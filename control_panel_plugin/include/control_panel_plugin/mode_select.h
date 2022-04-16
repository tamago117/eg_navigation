/**
* @file mode_select.h
* @brief rviz plugin for robot control
* @author Michikuni Eguchi
* @date 2021.12.10
*/

#pragma once

#ifndef Q_MOC_RUN
    #include <ros/ros.h>
#endif

#include <string>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>

namespace Ui {
class mode_selectUI;
}

namespace control_panel_plugin{

class modeSelect : public rviz::Panel
{
    Q_OBJECT
public:
    modeSelect(QWidget *parent = nullptr);
    ~modeSelect() override;

    void onInitialize() override;
    void onEnable();
    void onDisable();

private Q_SLOTS:
    void startButtonClicked();
    void stopButtonClicked();
    void dialValueChanged(int value);
    void poseEstimateButtonClicked();
    void wayPointNumberChanged(int value);
    void restartButtonClicked();
    void wpTopic_lineEditChanged();

private:
    void runPublish(const ros::TimerEvent& e);
    void stopPublish(const ros::TimerEvent& e);
    ros::Timer delay_timer;


protected:
    Ui::mode_selectUI* ui;
    std_msgs::String message;
    int delayTime{0};
    int wpNum{0};
    std::string topic_name;

    ros::NodeHandle nh;
    ros::Publisher mode_pub;
    ros::Publisher setWp_pub;
    ros::Publisher initial_pub;
};

}//namespace control_panel_plugin


