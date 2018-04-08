#include "kv01_driver/common_functions.h"
#include "kv01_driver/joint_speed.h"
#include <sensor_msgs/Joy.h>
#include "std_srvs/Empty.h"
#include "kv01_driver/driver.h"


double speedA[6] = {0, 0, 0, 0, 0, 0};
double speed_max[6] = {0.2, 0.15, 0.25, 0.25, 0.3, 0.5};
double prevGripper = 0;

class Kv01_joy {
public:
    Kv01_joy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    bool request_for_driver(int *joint, uint8_t command, uint8_t data);

    ros::NodeHandle n;

    int klb;
    std::string topic;
    int axes;
    bool flag_stlacene, brzdy_motory_on, brzdy_motory_off, button_comm;
    bool flag_button;
    ros::Publisher speed_pub;
    ros::Subscriber joy_sub;
    ros::ServiceClient driver_client, wifi_client;
    std_srvs::Empty empty_request;
};

Kv01_joy::Kv01_joy() {
    flag_stlacene = false;
    flag_button = false;
    n = ros::NodeHandle("~");

    n.param("topic", topic, topic);

    driver_client = n.serviceClient<kv01_driver::driver>("/kv01/driver_servis");
    wifi_client = n.serviceClient<std_srvs::Empty>("/kv01/wifi_restart");
    speed_pub = n.advertise<kv01_driver::joint_speed>("/kv01/joint_speed", 1);

    joy_sub = n.subscribe<sensor_msgs::Joy>(topic, 1, &Kv01_joy::joyCallback, this);
}

bool Kv01_joy::request_for_driver(int *joint, uint8_t command, uint8_t data) {
    kv01_driver::driver driver_client_data;
    for (int i = 0; i < 6; i++) {
        driver_client_data.request.joint[i] = joint[i];
    }
    brzdy_motory_on = false;
    brzdy_motory_off = false;
    button_comm = false;
    driver_client_data.request.command = command;
    driver_client_data.request.data = data;
    //driver_client.waitForExistence();
    if (driver_client.call(driver_client_data)) {
        return true;
    } else {
        // ROS_INFO("Niekde vznikla chyba");
        return false;
    }
}


void Kv01_joy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {

    kv01_driver::joint_speed joint_speed;
    joint_speed.command = comm_motory_speed;

    if (joy->buttons[0] == 0) {
        flag_stlacene = false;
        if (joy->buttons[1] == 1) {
            if (!flag_button) {
                for (int i = 0; i < 3; i++) {
                    joint_speed.joint = 3 + i;
                    joint_speed.speed = 0;
                    if (speedA[joint_speed.joint] != joint_speed.speed) {
                        speedA[joint_speed.joint] = joint_speed.speed;
                        speed_pub.publish(joint_speed);
                    }

                }
            }

            flag_button = true;
            for (int i = 0; i < 3; i++) {
                joint_speed.joint = 2 - i;
                if (joy->axes[i] > 0.15 && joy->axes[i] <= 0.4)
                    joint_speed.speed = speed_max[joint_speed.joint] / 4;
                if (joy->axes[i] < -0.15 && joy->axes[i] >= -0.4)
                    joint_speed.speed = -speed_max[joint_speed.joint] / 4;

                if (joy->axes[i] > 0.4 && joy->axes[i] <= 0.7)
                    joint_speed.speed = speed_max[joint_speed.joint] / 2;
                if (joy->axes[i] < -0.4 && joy->axes[i] >= -0.7)
                    joint_speed.speed = -speed_max[joint_speed.joint] / 2;

                if (joy->axes[i] > 0.7 && joy->axes[i] <= 1)
                    joint_speed.speed = speed_max[joint_speed.joint] - 0.02;

                if (joy->axes[i] < -0.7 && joy->axes[i] >= -1) {
                    joint_speed.speed = -speed_max[joint_speed.joint] + 0.02;
                }

                if (joy->axes[i] >= -0.15 && joy->axes[i] <= 0.15)
                    joint_speed.speed = 0;

                if (speedA[joint_speed.joint] != joint_speed.speed) {
                    speedA[joint_speed.joint] = joint_speed.speed;
                    speed_pub.publish(joint_speed);
                }

            }
        } else {
            if (flag_button) {
                for (int i = 0; i < 3; i++) {
                    joint_speed.joint = 2 - i;
                    joint_speed.speed = 0;
                    if (speedA[joint_speed.joint] != joint_speed.speed) {
                        speedA[joint_speed.joint] = joint_speed.speed;
                        speed_pub.publish(joint_speed);
                    }

                }
            }
            flag_button = false;
            for (int i = 0; i < 3; i++) {
                joint_speed.joint = 3 + i;

                if (joy->axes[i] > 0.15 && joy->axes[i] <= 0.4)
                    joint_speed.speed = speed_max[joint_speed.joint] / 4;
                if (joy->axes[i] < -0.15 && joy->axes[i] >= -0.4)
                    joint_speed.speed = -speed_max[joint_speed.joint] / 4;

                if (joy->axes[i] > 0.4 && joy->axes[i] <= 0.7)
                    joint_speed.speed = speed_max[joint_speed.joint] / 2;
                if (joy->axes[i] < -0.4 && joy->axes[i] >= -0.7)
                    joint_speed.speed = -speed_max[joint_speed.joint] / 2;

                if (joy->axes[i] > 0.7 && joy->axes[i] <= 1)
                    joint_speed.speed = speed_max[joint_speed.joint] - 0.02;

                if (joy->axes[i] < -0.7 && joy->axes[i] >= -1) {
                    joint_speed.speed = -speed_max[joint_speed.joint] + 0.02;
                }

                if (joy->axes[i] >= -0.15 && joy->axes[i] <= 0.15)
                    joint_speed.speed = 0;

                if (speedA[joint_speed.joint] != joint_speed.speed) {
                    speedA[joint_speed.joint] = joint_speed.speed;
                    speed_pub.publish(joint_speed);
                }
            }
        }

    } else {
        if (!flag_stlacene) {
            for (int i = 0; i < 6; i++) {
                speedA[i] = 0;
                joint_speed.joint = i;
                joint_speed.speed = 0;
                speed_pub.publish(joint_speed);
                flag_stlacene = true;
            }
        }
    }


    if (joy->buttons[2] == 1 && brzdy_motory_off == false) {
        int joint_message[6] = {1, 1, 1, 1, 1, 1};
        if (Kv01_joy::request_for_driver(joint_message, comm_message, comm_motory_en)) {
            joint_message[5] = 0;
            Kv01_joy::request_for_driver(joint_message, comm_brzdy, comm_brzdy_run);
        }
        brzdy_motory_off = true;
    } else
        brzdy_motory_off = false;


    if (joy->buttons[3] == 1 && brzdy_motory_on == false) {
        int joint_message[6] = {1, 1, 1, 1, 1, 0};
        if (Kv01_joy::request_for_driver(joint_message, comm_brzdy, comm_brzdy_stop)) {
            joint_message[5] = 1;
            Kv01_joy::request_for_driver(joint_message, comm_message, comm_motory_dis);
        }
        brzdy_motory_on = true;
    } else
        brzdy_motory_on = false;


    if (joy->buttons[4] == 1 && button_comm == false) {
        int joint_message[6] = {1, 1, 1, 1, 1, 1};
        Kv01_joy::wifi_client.call(Kv01_joy::empty_request);

        if (Kv01_joy::request_for_driver(joint_message, comm_message, comm_motory_en)) {
            joint_message[5] = 0;
            Kv01_joy::request_for_driver(joint_message, comm_brzdy, comm_brzdy_run);
        }

        button_comm = true;
    } else
        button_comm = false;

    joint_speed.command = comm_gripper_control;
    joint_speed.speed = 0;
    if (joy->axes[5]==1) {
        // Otvorenie grippera
//        std::cout << "gripper open" << std::endl;
        joint_speed.joint = 1;
    }
    else if(joy->axes[5]==-1) {
        // Zatvorenie grippera
//        std::cout << "gripper close" << std::endl;
        joint_speed.joint = 2;
    }
    else {
        // Zastavenie grippera
//        std::cout << "gripper stop" << std::endl;
        joint_speed.joint = 3;
    }
    if(prevGripper != joy->axes[5])
        speed_pub.publish(joint_speed);

    prevGripper = joy->axes[5];
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "kv01_joystick");
    Kv01_joy kv01_joy;
    ros::spin();
}
