#ifndef QML_MEDIATOR_H
#define QML_MEDIATOR_H

#include "ros/ros.h"
// #include "actionlib/client/simple_action_client.h"

#include "std_srvs/SetBool.h"
#include "black_maine_app/Confirmation.h"

#include "black_maine_app/DistributionActionResult.h"
#include "black_maine_app/DistributionActionFeedback.h"

#include <QDebug>
#include <QObject>
#include <QVariant>

class QMLMediator : public QObject {
  Q_OBJECT

  Q_PROPERTY(QString result READ result WRITE setResult NOTIFY resultChanged);
  Q_PROPERTY(QString target READ target WRITE setTarget NOTIFY targetChanged);

private:
  QString result_;
  QString target_;
  QString feedback_;

  ros::NodeHandle nh_;

  ros::ServiceClient confirm_client_;
  ros::ServiceClient init_pose_client_;

  ros::Publisher cancel_pub_;
  ros::Subscriber result_sub_;
  ros::Subscriber feedback_sub_;

  void ResultHandle(const black_maine_app::DistributionActionResult::ConstPtr& msg) {
    setResult(QString(msg->status.text.c_str()));
  }

  void FeedbackHandle(const black_maine_app::DistributionActionFeedback::ConstPtr& msg) {
    setTarget(QString(msg->feedback.goal.c_str()));
    setResult(QString(msg->feedback.goal_state.c_str()));
  }

public:
  QMLMediator(QObject* parent = nullptr) {
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/black_maine_distribution_server/cancel", 1000);
    result_sub_ = nh_.subscribe("/black_maine_distribution_server/result", 1000, &QMLMediator::ResultHandle, this);
    feedback_sub_ = nh_.subscribe("/black_maine_distribution_server/feedback", 1000, &QMLMediator::FeedbackHandle, this);

    init_pose_client_ = nh_.serviceClient<std_srvs::SetBool>("/init_pose");
    confirm_client_ = nh_.serviceClient<black_maine_app::Confirmation>("/confirmation");
    
    setTarget("--");
    setResult("Connected");
  }

  QString result() const{ return result_; }
  QString target() const{ return target_; }

  void setResult(const QString &value) {
    if(value != result_) {
      result_ = value;
      emit resultChanged();
    }
  }

  void setTarget(const QString &value) {
    if(value != target_) {
      target_ = value;
      emit targetChanged();
    }
  }

signals:
  void resultChanged();
  void targetChanged();

public slots:
  void confirm() {
    black_maine_app::Confirmation srv;
    srv.request.confirm = true;
    if(confirm_client_.call(srv)) {
      setResult("Task confirmed");
      ROS_INFO("Task confirmed");
    } else {
      setResult("Failed to confirm");
      ROS_ERROR("Failed to call service: confirm");
    }
  }

  void cancel() {
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.stamp = ros::Time(0,0);
    cancel_msg.id = "";
    cancel_pub_.publish(cancel_msg);
    ROS_INFO("Cancel all goals");
    setResult("Cancel goal");
  }

  void init_pose() {
    std_srvs::SetBool srv;
    srv.request.data = true;
    if(init_pose_client_.call(srv)) {
      setResult("Reset location success");
      ROS_INFO("Reset location success");
    } else {
      setResult("Failed to reset location");
      ROS_ERROR("Failed to call service: init_pose");
    }
  }
};

#endif
