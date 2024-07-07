#include <ros/ros.h>
#include <std_msgs/String.h>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

void publishKey(int fd, ros::Publisher& pub)
{
    struct input_event ev;
    ssize_t n;

    while (ros::ok())
    {
        n = read(fd, &ev, sizeof(struct input_event));
        if (n == (ssize_t)-1)
        {
            if (errno == EINTR)
                continue;
            else
                break;
        }
        else if (n != sizeof(struct input_event))
        {
            errno = EIO;
            break;
        }

        if (ev.type == EV_KEY && ev.value >= 0 && ev.value <= 2)
        {
            std_msgs::String msg;
            msg.data = std::to_string(ev.code);
            pub.publish(msg);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener");
    ros::NodeHandle nh;
    ros::Publisher key_pub = nh.advertise<std_msgs::String>("keyboard/keydown", 10);

    const char *device = "/dev/input/event3"; // Убедитесь, что это правильный путь к вашему устройству ввода
    int fd = open(device, O_RDONLY);
    if (fd == -1)
    {
        ROS_ERROR("Cannot open %s: %s.", device, strerror(errno));
        return EXIT_FAILURE;
    }

    publishKey(fd, key_pub);
    close(fd);
    return 0;
}