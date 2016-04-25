#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <sstream>

using namespace std;

uint8_t array_train_face[4] = {1,1,0,0};
uint8_t array_reco_face[4] = {1,3,0,0};
uint8_t array_train_body[4] = {2,1,0,0};
uint8_t array_reco_body[4] = {2,3,0,0};
uint8_t array_find_obj[4] = {3,3,0,0};
uint8_t array_exit[4] = {1,4,0,0};

int main(int argc, char **argv){
	ros::init(argc, argv, "int32_publisher");
	ros::NodeHandle n;
	ros::Publisher uint32_pub = n.advertise<std_msgs::Int32>("/andbot/fr_order", 1000);
	ros::Rate loop_rate(5);
	int option;	
	while (ros::ok()){	
	do{
		cout << "1) Start training face" << endl;
		cout << "2) Start recognition face" << endl;
		cout << "3) Start training body" << endl;
		cout << "4) Start recognition body" << endl;
		cout << "5) Start to find object" << endl;
		cout << "6) Exit Program " << endl;
		cout << "Please select an option : ";
		cin >> option;
		if(option == 1){
			cout << "Insert user_id " << endl;
			cin >> array_train_face[2];
			std_msgs::Int32 msg_train_face;
			msg_train_face.data = array_train_face[0] | (array_train_face[1] << 8) | (array_train_face[2] << 16) | (array_train_face[3] << 24);
			uint32_pub.publish(msg_train_face);
			ros::spinOnce();
		}
		else if(option == 2){
			std_msgs::Int32 msg_reco_face;
			msg_reco_face.data = array_reco_face[0] | (array_reco_face[1] << 8) | (array_reco_face[2] << 16) | (array_reco_face[3] << 24);
			uint32_pub.publish(msg_reco_face);
			ros::spinOnce();
		}
		else if(option == 3){
			cout << "Insert user_id " << endl;
			cin >> array_train_body[2];
			std_msgs::Int32 msg_train_body;
			msg_train_body.data = array_train_body[0] | (array_train_body[1] << 8) | (array_train_body[2] << 16) | (array_train_body[3] << 24);
			uint32_pub.publish(msg_train_body);
			ros::spinOnce();
		}
		else if(option == 4){
			std_msgs::Int32 msg_reco_body;
			msg_reco_body.data = array_reco_body[0] | (array_reco_body[1] << 8) | (array_reco_body[2] << 16) | (array_reco_body[3] << 24);
			uint32_pub.publish(msg_reco_body);
			ros::spinOnce();
		}
		else if(option == 5){
			std_msgs::Int32 msg_find_obj;
			msg_find_obj.data = array_find_obj[0] | (array_find_obj[1] << 8) | (array_find_obj[2] << 16) | (array_find_obj[3] << 24);
			uint32_pub.publish(msg_find_obj);
			ros::spinOnce();
		}
		else if(option == 6){
		cout << "Terminating Program" << endl;
			std_msgs::Int32 msg_exit;
			msg_exit.data = array_exit[0] | (array_exit[1] << 8) | (array_exit[2] << 16) | (array_exit[3] << 24);
			uint32_pub.publish(msg_exit);
			ros::spinOnce();
			loop_rate.sleep();
		}
		else{
		cout << "Invalid Option entered" << endl;
		}
	}while(option != 6);
	return 0;
	}
	return 0;
}
