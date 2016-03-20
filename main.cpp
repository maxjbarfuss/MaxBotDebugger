#include <iostream>
#include<fstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <termios.h>

#include <MessageBroker.h>
#include <StandardTypes.pb.h>
#include <Vector.pb.h>
#include <Velocity.pb.h>
#include <Range.pb.h>
#include <Pose.pb.h>

char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
            perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror ("tcsetattr ~ICANON");
    return (buf);
}

int main()
{
    std::ofstream myFile ("gyro.csv");
    MaxBotMessages::MessageBroker messageBroker(1);

//    messageBroker.Subscribe("POS2", [&](std::string s){
//        MaxBotMessages::Pose2Stamped p;
//        p.ParseFromString(s);
//        std::cout << std::setw(15) << p.pose().x() << "," << std::setw(15) << p.pose().y() << "," << std::setw(15) << p.pose().heading() << std::endl;
//        myFile << std::setw(15) << v.x() << "," << std::setw(15) << v.y() << "," << std::setw(15) << v.z() << std::endl;
//    });

//    messageBroker.Subscribe("RANG", [&](std::string s){
//        MaxBotMessages::RangeStamped r;
//        r.ParseFromString(s);
//        std::cout << std::setw(15) << r.range().value() << "," << r.stamp().component_id() << std::endl;
//        //myFile << std::setw(15) << v.x() << "," << std::setw(15) << v.y() << "," << std::setw(15) << v.z() << std::endl;
//    });

//    messageBroker.Subscribe("VOLT", [&](std::string s){
//        MaxBotMessages::DoubleStamped r;
//        r.ParseFromString(s);
//        std::cout << std::setw(15) << r.double_().value() << "," << r.stamp().component_id() << std::endl;
//        //myFile << std::setw(15) << v.x() << "," << std::setw(15) << v.y() << "," << std::setw(15) << v.z() << std::endl;
//    });

    bool exit = false;

    double x = 0, y = 0, z = 0;
    messageBroker.Subscribe("AHRS", [&](std::string s){
        if (exit) return;
        MaxBotMessages::Vector3Stamped v;
        v.ParseFromString(s);
        x = v.vector().x();
        y = v.vector().y();
        z = v.vector().z();
        std::cout << std::setw(15) << x << std::setw(15) << y << std::setw(15) << z << std::endl;
        myFile << std::setw(15) << x << "," << std::setw(15) << y << "," << std::setw(15) << z << std::endl;
    });

    double linear = 0;
    double angular = 0;
    double lastLinear = 0;
    double lastAngular = 0;
    bool updateVelocity = false;
    unsigned loopCount = 0;
    MaxBotMessages::Velocity2Stamped velocity;
    velocity.mutable_stamp()->set_component_id("DEBUG");

    std::thread keyboard ([&]{
        char c = ' ';
        int arrowPressed  = 0;
        while(!exit) {
            c=getch();
            int v = static_cast<int>(c);
            int lastArrowPressed = arrowPressed;
            if (v == 120) {
                exit = true;
            } else if ((v == 27) || (arrowPressed > 0 && v == 91)) {
                arrowPressed++;
            } else if (arrowPressed > 1 && v == 68) { //left
                angular += .314159 / 3;
            } else if (arrowPressed > 1 && v == 65) { //up
                linear += .01;
            } else if (arrowPressed > 1 && v == 67) { //right
                angular -= .314159 / 3;
            } else if (arrowPressed > 1 && v == 66) { //down
                linear -= .01;
            } else if ( v == 32 ) {
                linear = 0;
                angular = 0;
            }
            if (lastArrowPressed == arrowPressed)
                arrowPressed = 0;
        }
    });
    keyboard.detach();

    while(!exit) {
        if (lastLinear != linear || lastAngular != angular) {
            lastLinear = linear;
            lastAngular = angular;
            updateVelocity = true;
        }
        if (updateVelocity || ( loopCount++ % 20 == 0)) {
            velocity.mutable_stamp()->set_microseconds_since_epoch(messageBroker.MicrosecondsSinceEpoch());
            velocity.mutable_velocity()->set_linear(linear);
            velocity.mutable_velocity()->set_angular(angular);
            updateVelocity = false;
            messageBroker.Publish("TELE", velocity);
            //std::cout << "set to: " << linear << std::endl;
        }
        messageBroker.DoWork();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    myFile.close();
}
