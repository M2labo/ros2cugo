#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include <JetsonGPIO.h>

#include <chrono>
#include <memory>
#include <string>

#include "cugo_node/cugo.hpp"
using namespace GPIO;
using std::this_thread::sleep_for;

void help_print()
{
  printf("For cugo node : \n");
  printf("cugo [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    help_print();
    return 0;
  }

  rclcpp::init(argc, argv);


  rclcpp::executors::SingleThreadedExecutor executor;

  
  GPIO::setmode(GPIO::BOARD);
  GPIO::PWM pwm_right(32, 50);
  GPIO::PWM pwm_left(33, 50);
  
  pwm_right.start(7.5);
  pwm_left.start(7.5);
  sleep_for(std::chrono::seconds(5));
  auto cugo = std::make_shared<m2labo::cugo::Cugo>(pwm_right,pwm_left);
  executor.add_node(cugo);
  executor.spin();
  rclcpp::shutdown();
  pwm_right.stop();
  pwm_left.stop();
  GPIO::cleanup();

  return 0;
}
