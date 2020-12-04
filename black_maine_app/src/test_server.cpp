#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <black_maine_app/Confirmation.h>
#include <string>
#include <thread>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <black_maine_app/DistributionAction.h>
// #include <sound_play/SoundRequestAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::ActionServer<black_maine_app::DistributionAction> Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::ServerGoalHandle<black_maine_app::DistributionAction> GoalHandle;
// typedef actionlib::SimpleActionClient<sound_play::SoundRequestAction> SoundPlayClient;;
typedef actionlib::ManagedList<std::string> ManagedList;
bool move_goal_flag, sound_play_flag;

void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Move base feedback callback");
}

void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  move_goal_flag = true;
  ROS_INFO_STREAM("Move base done callback");
}


class PickTaskAction
{
public:
  ros::NodeHandle nh_;
  Server as_;
  ManagedList ml_;
  GoalHandle goal_handle_;
  MoveBaseClient move_base_client_;
  black_maine_app::DistributionFeedback feedback_;
  black_maine_app::DistributionResult result_;
	ros::ServiceServer next_server;

	int step;
	bool task_active_ = false, next_goal_ = false, waiting_confirm_ = false;

  geometry_msgs::Pose standby_position_, pick_position_, place_position_;

  PickTaskAction(std::string name) : 
    as_(nh_, name, boost::bind(&PickTaskAction::ExecuteCb, this, _1), boost::bind(&PickTaskAction::preemptCB, this, _1), false),
		move_base_client_("move_base", true)
  {
		// GetParam();
	  next_server = nh_.advertiseService("confirmation", &PickTaskAction::add, this);
    as_.start();
  }

  ~PickTaskAction(void)
  {
  }

	bool add(black_maine_app::Confirmation::Request  &req,
	         black_maine_app::Confirmation::Response &res)
	{
	  // 将输入参数中的请求数据相加，结果放到应答变量中
	  // res.sum = req.a + req.b;
	  // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	  // ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	  if (waiting_confirm_)
	  {
		 	ROS_INFO("confirmation server request");
		  next_goal_ = true;
		  waiting_confirm_ = false;
	  }

	  return true;
	}

	 
  void ExcuteThread(GoalHandle gh) {
		PickAndPlace(gh);
  }

	void ExecuteCb(GoalHandle gh)
	{
		// const std::string &ss = "string";
		// ml_.add(ss);

		// ROS_INFO("%i", ml_.begin());		
		// ml_.erase(ml_.begin());
		// ROS_INFO("%i", ml_.end());		
		ROS_INFO("````````````````````````````````````````");
	  ros::WallDuration(15.0).sleep();
		
		// if (task_active_)
		// {
		// 	gh.setRejected();
		//   ROS_INFO("new goal rejected");
		// 	return;
		// }

		// gh.setAccepted();
		// task_active_ = true;
		// ROS_INFO("%c", gh.getGoal()->places);
	 //  ROS_INFO("call pick task server");
		// // GetParam();
		// std::thread thread(&PickTaskAction::ExcuteThread, this, gh);
		// thread.detach();
		// task_active_ = false;
	}

  void preemptCB(GoalHandle gh)
  {
	  ROS_INFO("introduce server cancel!");
    
		// cancel move_base goal
    move_base_client_.cancelAllGoals();

    // set the action state to preempted
    task_active_ = false;
    gh.setCanceled();
  }

  // void GetParam() {
	 //  GetGoalPose("standby");
	 //  GetGoalPose("pick");
	 //  GetGoalPose("place");
  // }

	geometry_msgs::Pose GetGoalPose(std::string goal_name) {

	  std::vector<float> pose_vector(3), orientation_vector(4);
	  nh_.getParam("target_goal/"+goal_name+"/pose", pose_vector);
	  nh_.getParam("target_goal/"+goal_name+"/orientation", orientation_vector);

	  geometry_msgs::Pose pose;
	  pose.position.x = pose_vector[0];
	  pose.position.y = pose_vector[1];
	  pose.position.z = pose_vector[2];
	  pose.orientation.x = orientation_vector[0];
	  pose.orientation.y = orientation_vector[1];
	  pose.orientation.z = orientation_vector[2];
	  pose.orientation.w = orientation_vector[3];

	  return pose;
	  // if (goal_name == "standby") {
	  // 	standby_position_ = pose;
	  // } else if (goal_name == "pick") {
	  // 	pick_position_ = pose;
	  // } else if (goal_name == "place") {
	  // 	place_position_ = pose;
	  // }
	}

  void PickAndPlace(GoalHandle gh){
  	ROS_INFO("introduce");
  	std::vector<std::string> places_array = gh.getGoal()->places;
  	ROS_INFO("%i", places_array.size());
  	// ROS_INFO("%c", places_array[0].c_str());

  	for (int i = 0; i < places_array.size(); ++i)
  	{
  		std::string description = "Move to position " + places_array[i];
			// feedback_.step_description = description;
			// feedback_.step_index = 0;

		  geometry_msgs::Pose pose = GetGoalPose(places_array[i]);

			step = 1;
			bool loop = true;
			next_goal_ = false;
		  while(loop) {

				if (step == 1) {
					/* code */
					// feedback_.step_description = "Move to postion";
					// feedback_.step_index = 1;
					gh.publishFeedback(feedback_);
				  ros::WallDuration(5.0).sleep();
				  ROS_INFO("Move to standby goal");
				  // if (!MoveToGoal(pick_position_, gh))
				  // {
				  // 	return;
				  // }
					// feedback_.step_index = 2;
					// feedback_.step_description = "Wait for confirmation";
				}
				// else if(step == 2) {
				// 	gh.publishFeedback(feedback_);
				//   ros::WallDuration(1.0).sleep();
			 //    ROS_INFO("Introduce");
				// 	feedback_.step_index = 3;
				// 	feedback_.step_description = "Move to pick postion";
				// }

				else if (step == 2)
				{
				  ROS_INFO("step2");
				  waiting_confirm_ = true;
					while(ros::ok and gh.getGoalStatus().status != 2 and !next_goal_) {
					  ros::WallDuration(1.0).sleep();
					  ROS_INFO("wait for confirmation");
					}
				}

				if (gh.getGoalStatus().status == 2) {
					// step = 1;
					// gh.publishFeedback(feedback_);
					ROS_INFO("return");
					task_active_ = false;
					return;
				}
				else if(step == 2) {

					loop = false;
				}
				step += 1;
		  }
	  	ROS_INFO("%i", i);
  	}

	  // geometry_msgs::Pose position_standby = GetGoalPose("position_standby");
	  // if (!MoveToGoal(pose_standby, gh))
	  // {
	  // 	return;
	  // }

		// if (gh.getGoalStatus().status == 2) {
		// 	ROS_INFO("return");
		// 	return;
		// }

		// result_.finished_step = 7;
	  // gh.setSucceeded(result_, "all");
    if (gh.getGoalStatus().status == 1)
    {
		  gh.setSucceeded();
    }
	  // gh.setSucceeded();
	  task_active_ = false;
		ROS_INFO("set succeeded");
  }

	bool MoveToGoal(geometry_msgs::Pose pose, GoalHandle gh) {
	  move_base_msgs::MoveBaseGoal mb_goal;
	  mb_goal.target_pose.header.stamp = ros::Time::now();
	  mb_goal.target_pose.header.frame_id = "map";
	  mb_goal.target_pose.pose = pose;

	  move_goal_flag = false;
	  move_base_client_.sendGoal(mb_goal,
	            boost::bind(&MovebaseDoneCallback, _1, _2),
	            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
	            boost::bind(&MovebaseFeedbackCallback, _1));

	  while(move_goal_flag==false) {
	    ros::WallDuration(1.0).sleep();
	    ROS_INFO_STREAM("moving...");
	  }

    if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			  ROS_INFO_STREAM("get goal");
        // ROS_INFO("SUCCEEDED");
  		  return true;
  	}
    else {
        ROS_INFO("Cancel Goal!");
        move_base_client_.cancelAllGoals();
        if (gh.getGoalStatus().status == 1)
        {
				  gh.setAborted();
        }
        task_active_ = false;
    		return false;
    }
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_distribution_server");

  PickTaskAction averaging("turtlebot_distribution_server");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  // ros::spin();

  return 0;
}