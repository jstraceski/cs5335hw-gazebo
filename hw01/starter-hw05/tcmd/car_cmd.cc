
#include <iostream>
#include <string>
#include <vector>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using namespace gazebo;

int
main(int argc, char* argv[])
{
    if (argc != 3) {
        cout << "Usage: " << endl;
        cout << "\t" << argv[0] << " L R" << endl;
        return 1;
    }

    client::setup(argc, argv);
    transport::NodePtr node(new transport::Node());
    node->Init();

    int xx = std::atoi(argv[1]);
    int yy = std::atoi(argv[2]);
    cout << "xx,yy " << xx << "," << yy << endl;

    auto msg = msgs::ConvertAny(xx * 256 + yy);
    cout << "zz = " << msg.int_value() << endl;

    string topic = "~/tankbot0/vel_cmd";
    transport::PublisherPtr pub = node->Advertise<msgs::Any>(topic);

    pub->WaitForConnection();
    pub->Publish(msg);

    client::shutdown();
    return 0;
}
