#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QObject>
#include <QThread>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QTimer>
#include <QDebug>
#include <deque>
#include "struct.h"
#include "prasejson.h"
#include"CLOUD_TASK/cloud_task.h"
#include "plusgo_msgs/dispatch_cmd.h"
#include <geometry_msgs/PoseStamped.h>
#include"plusgo_msgs/VehicleLocation.h"
#include "plusgo_msgs/ByWireVehicleState.h"
#include "plusgo_msgs/basic_info.h"
#include "plusgo_msgs/SystemInfoReport.h"
#include "plusgo_msgs/middleware.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#define wait_time 1.0
const double deg_to_rad = 3.1415926535 / 180.0;
class RosWorker : public QObject
{
    Q_OBJECT
public:
    ~RosWorker();
    static RosWorker& getInstance();
    std::deque<struct Cloud> cloud;
    void publishMessage(const QString& message);
    void startSubscription();
    void sendSignal();
    void callMessage(const struct Cloud& cloud);
    void callMessage(const struct Station& station);
    void subMessage(const struct Cloud& cloud);
    void subscriberCallback(QByteArray jsonQString);
    void slam_Callback(const plusgo_msgs::VehicleLocation &msg);
    void pnc_Callback(const plusgo_msgs::SystemInfoReport &msg);
    void vehicle_state_Callback(const plusgo_msgs::ByWireVehicleState &msg);
    int instation = 0;
    bool go_next_station = false;
    int button_nextstation_cnt =0;
    int last_button_nextstation_state =1;
    bool first_start = true;

    void AssignCallback(const plusgo_msgs::middleware &message);
signals:
    void receivedMessage();
    void sendstation();
    void HMINextStation();
    void changeRoute();
    void sendlasteststation();
private slots:
    void spin();

protected:
    explicit RosWorker(QObject *parent = nullptr);

private:

    static RosWorker* instance_;
    ros::NodeHandle *nh_;
    ros::ServiceClient client_;
    ros::Publisher relocation_pub;
    ros::Subscriber slam_subscriber;
    ros::Subscriber vehicle_state_subscriber;
    ros::Subscriber pnc_subscriber;
    static int cmdid;
    plusgo_msgs::dispatch_cmd send_cmd;
    bool emit_to_main = true;

    ros::Publisher io_publisher;
    std_msgs::String io_msg;

    ros::Subscriber sub_assign;


};

#endif // ROSWORKER_H
