/**
 * scan_database.cpp
 *
 * Database node to store detections of people and workspaces. Information is sent to the database node through the
 * /tag_scan topic using a Scan msg format.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#include "turtlebot3_disinfection/scan_database.h"

/**
 * Class constructor.
 *
 * @param nh ROS NodeHandle object used to instantiate this class as a ROS node.
 */
ScanDatabase::ScanDatabase(ros::NodeHandle* nh)
    : sub_tag_scan_(nh->subscribe("/tag_scan", 1000, &ScanDatabase::tagScanCallback, this))
{
}

/**
 * Class destructor which frees dynamic memory allocations used in the database.
 */
ScanDatabase::~ScanDatabase()
{
    // Iterate through and delete all items in the workspace database
    for (auto it=workspace_database_.begin(); it!=workspace_database_.end(); it++)
    {
        delete it->second;
    }

    // Iterate through and delete all items in the person database
    for (auto it=person_database_.begin(); it!=person_database_.end(); it++)
    {
        delete it->second;
    }
}

/**
 * Prints the list of all workspaces in the database and details about each workspace.
 */
void ScanDatabase::printWorkspaces() const
{
    std::cout << "------------------------WORKSPACES------------------------" << std::endl;

    // Iterate through all workspaces in the database
    for (auto it=workspace_database_.begin(); it!=workspace_database_.end(); it++)
    {
        Workspace* workspace = it->second;

        // Print information about the workspace
        std::cout << "Workspace ID: " << workspace->tag_id << std::endl;
        std::cout << "Location: " << workspace->detection_locations[workspace->detection_locations.size()-1].x << ", "
            << workspace->detection_locations[workspace->detection_locations.size()-1].y << ", "
            << workspace->detection_locations[workspace->detection_locations.size()-1].z << std::endl;
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

/**
 * Prints the list of all people in the database and details about each person.
 */
void ScanDatabase::printPeople() const
{
    std::cout << "------------------------PEOPLE------------------------" << std::endl;

    // Iterate through all people in the database
    for (auto it=person_database_.begin(); it!=person_database_.end(); it++)
    {
        Person* person = it->second;

        // Print information about the person
        std::cout << "Person ID: " << person->tag_id << std::endl;
        std::cout << "Person Name: " << person->name << std::endl;
        std::cout << "Last Seen Workspace ID:" << person->last_seen_workspace_id << std::endl;
        std::cout << "Detection Times:" << std::endl;
        for (auto& time : person->detection_times)
        {
            std::cout << "\t- " << time << std::endl;
        }
        std::cout << std::endl;
    }
}

/**
 * Prints the list of all scans in the database and details about each scan.
 */
void ScanDatabase::printScans()
{
    std::cout << "------------------------SCANS------------------------" << std::endl;
    // Variable to show the index of each scan
    int scan_index = 0;

    // Iterate through all scans in the database
    for (auto& scan_msg : scan_list_)
    {
        // Print information about the scan
        std::cout << "Scan ID: " << scan_index << std::endl;
        std::cout << "Scan Time: " << scan_msg.scan_end_time;
        std::cout << "Workspace ID: " << scan_msg.workspace_id << std::endl;
        std::cout << "People:" << std::endl;
        for (auto& person_id : scan_msg.people_ids)
        {
            std::cout << "\t- " << person_database_[person_id]->name << std::endl;
        }
        std::cout << "Disinfected: " << (bool)scan_msg.disinfected << std::endl;
        std::cout << std::endl;

        scan_index++;
    }
}

/**
 * Callback function which is called when a Scan msg is received. This function uses the information stored in the Scan
 * message to instantiate Person and Workspace objects to store in the database.
 *
 * @param msg The Scan msg containing information about the scan.
 */
void ScanDatabase::tagScanCallback(const turtlebot3_disinfection::Scan& msg)
{
    std::cout << "NEW SCAN RECEIVED" << std::endl;

    // Add the scan to the scan list
    scan_list_.push_back(msg);

    // Attempt to find the workspace in the database map
    auto workspace_iterator = workspace_database_.find(msg.workspace_id);

    // Pointer to workspace object
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

    // Update the information stored about the workspace
    workspace->detection_times.push_back(msg.workspace_pose.header.stamp);
    workspace->detection_locations.push_back({(float)msg.workspace_pose.pose.position.x,
                                              (float)msg.workspace_pose.pose.position.y,
                                              (float)msg.workspace_pose.pose.position.z});

    // Add a disinfection time if a disinfection was performed
    if (msg.disinfected)
    {
        workspace->disinfection_times.push_back(msg.scan_end_time.data);
    }

    // Insert the workspace into the map
    workspace_database_.insert(std::make_pair(workspace->tag_id, workspace));

    // Get the number of people in the scan
    int n_people = msg.people_ids.size();

    // Iterate through all people in the scan
    for (int i=0; i<n_people; i++)
    {
        // Attempt to find the person in the database map
        auto person_iterator = person_database_.find(msg.people_ids[i]);

        // Pointer to person object
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

        // Update the information stored about the person
        person->detection_times.push_back(msg.people_poses[i].header.stamp);
        person->detection_locations.push_back({(float)msg.people_poses[i].pose.position.x,
                                                  (float)msg.people_poses[i].pose.position.y,
                                                  (float)msg.people_poses[i].pose.position.z});
        person->last_seen_workspace_id = workspace->tag_id;

        // Insert the person into the map
        person_database_.insert(std::make_pair(person->tag_id, person));
    }
}

/**
 * This routine is run as in a separate thread, and constantly polls the user for input commands corresponding to
 * database queries. The database queries are given by the following inputs:
 * "p" - Prints the list of people
 * "w" - Prints the list of workspaces
 * "s" - Prints the list of scans
 *
 * @param database Pointer to the database object. This is used to call the member functions of the database.
 */
void userInputLoop(ScanDatabase* database)
{
    while (true)
    {
        // std::cout << "[database]>> ";

        // Get input from the user
        std::string user_input;
        std::cin >> user_input;

        // Perform action based on the user command
        if (user_input == "p")
        {
            database->printPeople();
        }
        else if (user_input == "w")
        {
            database->printWorkspaces();
        }
        else if (user_input == "s")
        {
            database->printScans();
        }
    }
}


int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "scan_database");

    // Create a node handle to pass to the class constructor
    ros::NodeHandle nh;

    // Instantiate the class
    ScanDatabase scan_database(&nh);

    // Create a thread to run the userInputLoop function
    boost::thread user_input_thread(userInputLoop, &scan_database);

    // Wait for and process callbacks
    ros::spin();

    // Interrupt the user_input_thread execution
    user_input_thread.interrupt();

    return 0;
}
