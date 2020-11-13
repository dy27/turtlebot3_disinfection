
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
    std::cout << "------------------------WORKSPACES------------------------" << std::endl;

    for (auto it=workspace_database_.begin(); it!=workspace_database_.end(); it++)
    {
        Workspace* workspace = it->second;
        std::cout << "Workspace ID: " << workspace->tag_id << std::endl;
        std::cout << "Visit Times:" << std::endl;
        for (auto& time : workspace->detection_times)
        {
            std::cout << "\t- " << time << std::endl;
        }
        std::cout << "Disinfection Times:" << std::endl;
        for (auto& time : workspace->disinfection_times)
        {
            std::cout << "\t- " << time << std::endl;
        }
        std::cout << std::endl;
    }
}


void DisinfectionDatabase::printPeople()
{
    std::cout << "------------------------PEOPLE------------------------" << std::endl;

    for (auto it=person_database_.begin(); it!=person_database_.end(); it++)
    {
        Person* person = it->second;
        std::cout << "Person ID: " << person->tag_id << std::endl;
        std::cout << "Person Name: " << person->name << std::endl;
        std::cout << "Last Seen Workspace ID:" << person->last_seen_workspace_id << std::endl;
        std::cout << std::endl;
    }
}


void DisinfectionDatabase::printScans()
{
    std::cout << "------------------------SCANS------------------------" << std::endl;
    int scan_index = 0;
    for (auto& scan_msg : scan_list_)
    {
        std::cout << "Scan ID: " << scan_index << std::endl;
        std::cout << "Scan Time: " << scan_msg.scan_end_time;
        std::cout << "Workspace ID: " << scan_msg.workspace_id << std::endl;
        std::cout << "People:" << std::endl;
        for (auto& person_id : scan_msg.people_ids)
        {
            std::cout << "\t- " << person_database_[person_id]->name << std::endl;
        }
        std::cout << "Disinfected: " << scan_msg.disinfected << std::endl;
        std::cout << std::endl;

        scan_index++;
    }
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
    if (msg.disinfected)
    {
        workspace->disinfection_times.push_back(msg.scan_end_time.data);
    }

    workspace_database_.insert(std::make_pair(workspace->tag_id, workspace));


    int n_people = msg.people_ids.size();
    ROS_INFO("n_people: %d", n_people);
    ROS_INFO("n_people: %d", msg.people_poses.size());
    for (int i=0; i<n_people; i++)
    {
        // Attempt to find the person in the database map
        auto person_iterator = person_database_.find(msg.people_ids[i]);

        Person* person;

        // If the person is not in the database
        if (person_iterator == person_database_.end())
        {
            // Create a new person
            person = new Person(msg.people_ids[i]);
        }
        else
        {
            // Get the person that the iterator is pointing at
            person = person_iterator->second;
        }

        // Update the person
        person->detection_times.push_back(msg.people_poses[i].header.stamp);
        person->detection_locations.push_back({(float)msg.people_poses[i].pose.position.x,
                                                  (float)msg.people_poses[i].pose.position.y,
                                                  (float)msg.people_poses[i].pose.position.z});
        person->last_seen_workspace_id = workspace->tag_id;

        person_database_.insert(std::make_pair(person->tag_id, person));
    }
}

void userInputLoop(DisinfectionDatabase* database)
{
    while (true)
    {
        // std::cout << "[database]>> ";

        std::string user_input;
        std::cin >> user_input;

        if (user_input == "people")
        {
            database->printPeople();
        }
        else if (user_input == "workspaces")
        {
            database->printWorkspaces();
        }
        else if (user_input == "scans")
        {
            database->printScans();
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "disinfection_database");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    DisinfectionDatabase disinfection_database(&nh);

    boost::thread user_input_thread(userInputLoop, &disinfection_database);

    ros::spin();

    user_input_thread.interrupt();

    return 0;
}
