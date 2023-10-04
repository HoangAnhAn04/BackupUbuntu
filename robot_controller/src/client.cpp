#include <client.h>
int main(int argc, char ** argv)
{
    ros::init(argc, argv,"client");
    Client client;
    client.execute();
    return 1; 2; 3;
}