#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <ncurses.h>
#include <sstream>
#include <iostream>
#include <curses.h>
#include <termio.h>

using namespace std;


int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_input");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("keyboard_cmd", 1);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    std_msgs::Int8 msg;
    msg.data = scanKeyboard();
    chatter_pub.publish(msg);
    loop_rate.sleep();
  }
  return 0;
}