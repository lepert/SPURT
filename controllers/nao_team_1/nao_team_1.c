/*
 * File:         nao_team_1.c
 * Description:  This is a trivial C controller example
 *               It's purpose is to demonstrates a simple walk in the case Java virtual machine is not installed
 *               You can safely remove this source file and the corresponding executable file if you want to work on the Java example.
 * Author:       yvan.bourquin - www.cyberbotics.com
 */

#include <webots/robot.h>
#include <webots/utils/motion.h>

int main() {
  wb_robot_init();

  int time_step = wb_robot_get_basic_time_step();

  // load and start forward motion
  WbMotionRef forwards = wbu_motion_new("../../motions/Forwards50.motion");
  wbu_motion_set_loop(forwards, true);
  wbu_motion_play(forwards);

  // forever
  for (;;)
    wb_robot_step(time_step);

  // never reached
  return 0;
}
