#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <iostream>
#include <string>
#include <people_msgs/Person.h>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
ros::Publisher pub_tracked_people;

void sub_tracked_people_callback(spencer_tracking_msgs::TrackedPersons persons) {
    people_msgs::Person peoplePerson;
    int i = 0;
    std::string name;
    for (auto person : persons.tracks) {
        if (person.is_matched)
            {
            name = "fede" + person.track_id;
            peoplePerson.position.x = person.pose.pose.position.x;
            peoplePerson.position.y = person.pose.pose.position.y;
            peoplePerson.position.z = person.pose.pose.position.z;
            peoplePerson.velocity.x = person.twist.twist.linear.x;
            peoplePerson.velocity.y = person.twist.twist.linear.y;
            peoplePerson.velocity.z = person.twist.twist.linear.z;
            peoplePerson.reliability = 1.0;
            peoplePerson.name = name;
            pub_tracked_people.publish(peoplePerson);
            }
        }
    }


int main(int argc, char** argv)
    {

    putenv((char*)"ROS_NAMESPACE=locobot");
    ros::init(argc, argv, "tracked_people_translator");


    ros::NodeHandle node_handle;
    ros::Subscriber sub_tracked_people = node_handle.subscribe("/spencer/perception/tracked_persons", 1, sub_tracked_people_callback);

    pub_tracked_people = node_handle.advertise<people_msgs::Person>("/people", 1);

    ros::spin();
    return 0;
    }

