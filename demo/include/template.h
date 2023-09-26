#include "ros/ros.h"
class Template
{
    private://biến dùng trong
        ros::NodeHandle nh;
        int a;
    public://biến dùng ngoài
        Template()
        {
            this -> a=nh.getParam("a",this->a);
        }
        ~Template()
        {
        }
        void execute()
        {
            std::cout << this -> a;

        }
 //có thể viết code của các hàm trong hoặc viết 1 file cc bên ngoài
};