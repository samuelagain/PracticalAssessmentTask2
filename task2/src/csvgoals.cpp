#include <ros/ros.h>
#include <fstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdio.h>
#include <string> 
#include <mutex>

double * x_pos;
double * y_pos;
std::mutex coords_mutex;

//function to read from a csv file into a 2D Vector, Each vector is <x,y>
std::vector<std::vector<double> > parseCSV( std::string filename)
{
    std::ifstream  data(filename);
    std::string line;
    std::vector<std::vector<double> > parsedCsv;
    while(std::getline(data,line))
    {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> parsedRow;
        for(int i = 0; i <2 ; i++){
            std::getline(lineStream,cell,',');
            parsedRow.push_back(std::stod(cell));
        }

        parsedRow.push_back((double) 0);

        parsedCsv.push_back(parsedRow);
    }
    return parsedCsv;
}

//function to overwrite the coordinates.csv file with the coordinates and their outcome
void saveCoords( std::vector<std::vector<double>> coordsList, std::string filename)
{
   std::ofstream CsvFile(filename);
   //rewrite the coordinates visted
  for (int i=0;i<coordsList.size();i++)
  {
    std::string x  = std::to_string(coordsList[i][0]);
    std::string y  = std::to_string(coordsList[i][1]);
    CsvFile << x << "," << y;

    //record outcome in third column
    //greater than 0.5 (ie 1) is a success
    //less than -0.5 (ie -1) is a failure
    //if the default remains (0) than it is assumed the coordinate was not attempted
    if(coordsList[i][2] > 0.5){
        CsvFile << ", success\n";
    }else if(coordsList[i][2] < -0.5)
    {
        CsvFile << ", fail\n";
    }else{
        CsvFile << ", unattempted\n";;
    }
  }
  CsvFile.close();
}


//Callback function to overwrite global shared-memory variables of robot coords
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //use lock guard to prevent double read write on x_pos and y_pos
    const std::lock_guard<std::mutex> lock(coords_mutex);
    *x_pos = msg->pose.pose.position.x;
    *y_pos = msg->pose.pose.position.y;
}

int main(int argc, char** argv)
{
    //Global coordinates stored in shared memory to be access by odemtry callback and main process
    x_pos = (double *) mmap(NULL, sizeof(double), PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS, -1, 0); 
    y_pos = (double *) mmap(NULL, sizeof(double), PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS, -1, 0); 

    if(fork()==0){
         //forked process to maintain odemtry subscriber callback 
        ros::init(argc, argv, "odomtracker");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/odometry/filtered", 1000, odomCallback);
        ros::spin();
    }
    else{
        //forked process to maintain movebase server for goal seeking
    
        //set robot coord variables to zero incase telemetry fails to report coords
        *x_pos = 0;
        *y_pos = 0;

        ros::init(argc, argv, "simple_goal_seeker");

        std::string filename = "coordinates.csv";
        std::vector<std::vector<double>> coordslist = parseCSV(filename);

        //ROS_INFO("Coords_list loaded!");

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

        MoveBaseClient ac("move_base", true);

        while(!ac.waitForServer(ros::Duration(5.0))){
            //ROS_INFO("Waiting for the move_base action server to come up");
        }
        //ROS_INFO("Connected!");

        //iterate through goal coordinates and attempt to reach each with move base server
        for(int i = 0; i < coordslist.size(); i++){

            //intiilaze and set up goal
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "base_link";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = coordslist[i][0] - *x_pos;
            goal.target_pose.pose.position.y = coordslist[i][1] - *y_pos;
            goal.target_pose.pose.orientation.w = 1.0;

            //send goal to server
            //ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            //wait for the robot to do its thing (hopefully get to the coordinate)
            ac.waitForResult();

            //once coordinate is reached or movebase gives up, the outcome is saved in the third item in each vector
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("Successfully reach Coordinate!");
                coordslist[i][2] = (double) 1;
            }
            else
            {
                ROS_INFO("Failed to reach Coordinate!");
            coordslist[i][2] = (double) -1;
            }
        }

        //after all coordinated have been attempted, the csv is rewritten with outcomes and saved
        saveCoords( coordslist, filename);
    }

    return 0;
}