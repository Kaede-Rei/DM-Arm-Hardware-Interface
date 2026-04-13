#include "dm_hw/damiao.h"
#include "unistd.h"
#include <cmath>
#include <chrono>

damiao::Motor M1(damiao::DMH3510, 0x01, 0x00);
damiao::Motor M2(damiao::DM4310, 0x02, 0x00);
std::shared_ptr<SerialPort> serial;
damiao::MotorControl dm(serial);


int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = damiao::MotorControl(serial);

    dm.add_motor(&M1);
    // dm.addMotor(&M2);
    dm.disable(M1);
    // dm.disable(M2);
    sleep(1);
    if(dm.switch_control_mode(M1, damiao::VEL_MODE))
        std::cout << "Switch to VEL_MODE Success" << std::endl;
      // if(dm.switch_control_mode(M1, damiao::MIT_MODE))
      //   std::cout << "Switch to MIT Success" << std::endl;
      // if(dm.switch_control_mode(M2, damiao::POS_VEL_MODE))
      //   std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
      // std::cout<<"motor1 PMAX:"<<dm.read_motor_param(M1, damiao::PMAX)<<std::endl;
      // std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
      // std::cout<<"motor2 PMAX:"<<dm.read_motor_param(M2, damiao::PMAX)<<std::endl;
      // std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
      // if(dm.change_motor_param(M1, damiao::UV_Value, 12.6f))
      //   std::cout << "Change UV_Value Success" << std::endl;
      // std::cout<<"motor1 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
      // if(dm.change_motor_param(M2, damiao::UV_Value, 12.6f))
      //   std::cout << "Change UV_Value Success" << std::endl;
      // std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
      // if(dm.change_motor_param(M1,damiao::CTRL_MODE,1))
      //   std::cout << "Change CTRL_MODE Success" << std::endl;
      // std::cout<<"motor1 CTRL_MODE:"<<dm.read_motor_param(M1, damiao::CTRL_MODE)<<std::endl;
    dm.save_motor_param(M1);
    // dm.save_motor_param(M2);
    dm.enable(M1);
    // dm.enable(M2);
    sleep(1);
    while(1) {
        float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
        // dm.control_mit(M1, 30, 0.3, q*10, 0, 0);
        // dm.control_vel(M1, q * 100);
        // dm.control_pos_vel(M2, q*10,10);
        (void)q;

        dm.refresh_motor_status(M1);
        dm.refresh_motor_status(M2);
        std::cout << "motor1--- POS:" << M1.get_position() << " VEL:" << M1.get_velocity() << " CUR:" << M1.get_tau() << std::endl;
        // std::cout<<"motor2--- POS:"<<M2.get_position()<<" VEL:"<<M2.get_velocity()<<" CUR:"<<M2.get_tau()<<std::endl;
        usleep(1000);
        // std::cout<<"motor1 pos:"<<M1.get_position()<<std::endl;

    }



    return 0;
}
