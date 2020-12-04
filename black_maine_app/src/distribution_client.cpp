#include <black_maine_app/DistributionAction.h>
//#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

//这样定义下会用起来简洁许多
typedef actionlib::SimpleActionClient<black_maine_app::DistributionAction> Client;

class DistributionActionClient {
private:
    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState& state,
            const black_maine_app::DistributionResultConstPtr& result) {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Toal dish cleaned: %i", result->finished_step);
        ros::shutdown();
    }

    // 当目标激活的时候，会调用一次
    void ActiveCb() {
        ROS_INFO("Goal just went active");
    }

    // 接收服务器的反馈信息
    void FeedbackCb(
            const black_maine_app::DistributionFeedbackConstPtr& feedback) {
        // ROS_INFO("Got Feedback Complete Rate: %f", feedback->step);
    }
public:
    DistributionActionClient(const std::string client_name, bool flag = true) :
            client(client_name, flag) {
    }

    //客户端开始
    void Start() {
        //等待服务器初始化完成
        client.waitForServer();
        //定义要做的目标
        black_maine_app::DistributionGoal goal;
        goal.places.push_back("m1");
        goal.places.push_back("t1");
        goal.places.push_back("_standby");
        goal.confirm_list.push_back("True");
        goal.confirm_list.push_back("True");
        goal.confirm_list.push_back("False");
        //发送目标至服务器
        client.sendGoal(goal,
                boost::bind(&DistributionActionClient::DoneCb, this, _1, _2),
                boost::bind(&DistributionActionClient::ActiveCb, this),
                boost::bind(&DistributionActionClient::FeedbackCb, this, _1));
        //等待结果，时间间隔5秒
        client.waitForResult(ros::Duration(40.0));

        //根据返回结果，做相应的处理
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // printf("Yay! The dishes are now clean");
            ROS_INFO("SUCCEEDED");
        else {
            ROS_INFO("Cancel Goal!");
            client.cancelAllGoals();
        }

        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    }
private:
    Client client;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distribution_client");
    DistributionActionClient actionclient("black_maine_distribution_server", true);
    //启动客户端
    actionclient.Start();
    ros::spin();
    return 0;
}
