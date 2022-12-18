/**
* @file mode_select.cpp
* @brief rviz plugin for robot control
* @author Michikuni Eguchi
* @date 2021.12.10
*/

#include <pluginlib/class_list_macros.h>

#include <control_panel_plugin/mode_select.h>
#include <eg_navigation/robot_status.h>
#include "ui_mode_select.h"

namespace control_panel_plugin
{

modeSelect::modeSelect(QWidget *parent) : Panel(parent), ui(new Ui::mode_selectUI())
{
    ui->setupUi(this);

    /*pubpose.pose.covariance[0]=0.25;
    pubpose.pose.covariance[7]=0.25;
    pubpose.pose.covariance[35]=0.06853891945200942;*/
}

modeSelect::~modeSelect() = default;

void modeSelect::onInitialize()
{
    connect(ui->run, SIGNAL(clicked()), this, SLOT(startButtonClicked()));
    connect(ui->stop, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    connect(ui->dial_delay, SIGNAL(valueChanged(int)), this, SLOT(dialValueChanged(int)));
    connect(ui->poseEstimate, SIGNAL(clicked()), this, SLOT(poseEstimateButtonClicked()));
    connect(ui->spinBox_wayPoint, SIGNAL(valueChanged(int)), this, SLOT(wayPointNumberChanged(int)));
    connect(ui->restart, SIGNAL(clicked()), this, SLOT(restartButtonClicked()));
    connect(ui->lineEdit_wpTopic, SIGNAL(textChanged(const QString &)), this, SLOT(wpTopic_lineEditChanged()));
    //connect(ui-> (qt object name) , SIGNAL( object action ), this, SLOT( function ));

    mode_pub = nh.advertise<std_msgs::String>("mode_select/mode", 1);
    setWp_pub = nh.advertise<std_msgs::Int32>("wayPoint/set", 1);
    parentWidget()->setVisible(true);
}

void modeSelect::onEnable()
{
    show();
    parentWidget()->show();
}

void modeSelect::onDisable()
{
    hide();
    parentWidget()->hide();
}

void modeSelect::startButtonClicked()
{
    ROS_INFO("command 'run' is sent after %d s",delayTime);
    delay_timer = nh.createTimer(ros::Duration(delayTime), &modeSelect::runPublish, this, true);
}

void modeSelect::runPublish(const ros::TimerEvent& e)
{
    message.data = robot_status_str(robot_status::run);
    mode_pub.publish(message);
    //delay_timer.stop();

    ROS_INFO("mode : run");
}

void modeSelect::stopButtonClicked()
{
    ROS_INFO("command 'stop' is sent after %d s",delayTime);
    delay_timer = nh.createTimer(ros::Duration(delayTime), &modeSelect::stopPublish, this, true);
}

void modeSelect::stopPublish(const ros::TimerEvent& e)
{
    message.data = robot_status_str(robot_status::stop);
    mode_pub.publish(message);
    //run_timer.stop();

    ROS_INFO("mode : stop");
}

void modeSelect::dialValueChanged(int value)
{
    ui->lcdNumber_delay->display(value);
    delayTime = value;
    ROS_INFO("next command is sent after %d s",value);
}

void modeSelect::poseEstimateButtonClicked()
{
    ROS_INFO("initial pose estimate");
    static std_msgs::Int32 wpNumPub;
    wpNumPub.data = 0;

    setWp_pub.publish(wpNumPub);
}

void modeSelect::wayPointNumberChanged(int value)
{
    wpNum = value;
}

void modeSelect::restartButtonClicked()
{
    //service的ななにかでやりたい(int wpNum : Empty)
    ROS_INFO("restart");
    static std_msgs::Int32 wpNumPub;
    wpNumPub.data = wpNum;

    setWp_pub.publish(wpNumPub);

}

void modeSelect::wpTopic_lineEditChanged()
{
    std::string old_topic_name = topic_name;
    if(ui->lineEdit_wpTopic->text().isEmpty()){
        topic_name = "wayPoint/set";
    }else{
        topic_name = ui->lineEdit_wpTopic->text().toStdString();
    }

    ROS_INFO("You set the way point number topic name : %s", topic_name.c_str());

    if(old_topic_name != topic_name){
        setWp_pub = nh.advertise<std_msgs::Int32>(topic_name, 1);
    }
}

}//namespace control_panel_plugin

PLUGINLIB_EXPORT_CLASS(control_panel_plugin::modeSelect, rviz::Panel )

