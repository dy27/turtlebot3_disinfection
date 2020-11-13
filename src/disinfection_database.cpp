
#include "turtlebot3_disinfection/disinfection_database.h"


DisinfectionDatabase::DisinfectionDatabase(ros::NodeHandle* nh)
    : sub_tag_scan_(nh->subscribe("/tag_scan", 1000, &DisinfectionDatabase::tagScanCallback, this))
{
}

DisinfectionDatabase::~DisinfectionDatabase()
{
    for (auto it=workspace_database_.begin(); it!=workspace_database_.end(); it++)
    {
        delete it->second;
    }

    for (auto it=person_database_.begin(); it!=person_database_.end(); it++)
    {
        delete it->second;
    }
}

void DisinfectionDatabase::printWorkspaces()
{

}


void DisinfectionDatabase::printPeople()
{

}


void DisinfectionDatabase::printScanLog()
{

}

// void DisinfectionDatabase::updateWorkspace(const turtlebot3_disinfection::Scan& msg);
//
// void DisinfectionDatabase::updatePeople(const turtlebot3_disinfection::Scan& msg);

void DisinfectionDatabase::tagScanCallback(const turtlebot3_disinfection::Scan& msg)
{
    // Add the scan to the scan list
    // scan_list_.push_back(ScanEntry(msg));
    scan_list_.push_back(msg);

    // Attempt to find the workspace in the database map
    auto workspace_iterator = workspace_database_.find(msg.workspace_id);

    Workspace* workspace;

    // If the workspace is not in the database
    if (workspace_iterator == workspace_database_.end())
    {
        // Create a new workspace
        workspace = new Workspace(msg.workspace_id);
    }
    else
    {
        // Get the workspace that the iterator is pointing at
        workspace = workspace_iterator->second;
    }

    // Update the workspace
    workspace->detection_times.push_back(msg.workspace_pose.header.stamp);
    workspace->detection_locations.push_back({(float)msg.workspace_pose.pose.position.x,
                                              (float)msg.workspace_pose.pose.position.y,
                                              (float)msg.workspace_pose.pose.position.z});
    workspace->disinfection_times.push_back(msg.scan_end_time.data);


    int n_people = msg.people_ids.size();
    for (int i=0; i<n_people; i++)
    {
        // Attempt to find the person in the database map
        auto person_iterator = person_database_.find(msg.people_ids[i]);

        Person* person;

        // If the workspace is not in the database
        if (person_iterator == person_database_.end())
        {
            // Create a new workspace
            person = new Person(msg.people_ids[i]);
        }
        else
        {
            // Get the workspace that the iterator is pointing at
            person = person_iterator->second;
        }

        // Update the workspace
        workspace->detection_times.push_back(msg.people_poses[i].header.stamp);
        workspace->detection_locations.push_back({(float)msg.people_poses[i].pose.position.x,
                                                  (float)msg.people_poses[i].pose.position.y,
                                                  (float)msg.people_poses[i].pose.position.z});
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "disinfection_database");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    DisinfectionDatabase disinfection_database(&nh);

    ros::spin();

    return 0;
}
