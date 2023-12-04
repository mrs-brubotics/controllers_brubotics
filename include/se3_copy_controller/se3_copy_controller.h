#ifndef CONTROLLERS_BRUBOTICS_SE3_COPY_CONTROLLER_H
#define CONTROLLERS_BRUBOTICS_SE3_COPY_CONTROLLER_H

namespace controllers_brubotics
{

namespace se3_copy_controller
{

/* structs //{ */ /*//{*/
typedef struct
{
  // uav gains:
  double kpxy;           // position xy gain
  double kvxy;           // velocity xy gain
  double kaxy;           // acceleration xy gain (feed forward, =1)
  double kiwxy;          // world xy integral gain
  double kibxy;          // body xy integral gain
  double kiwxy_lim;      // world xy integral limit
  double kibxy_lim;      // body xy integral limit
  double kpz;            // position z gain
  double kvz;            // velocity z gain
  double kaz;            // acceleration z gain (feed forward, =1)
  double km;             // mass estimator gain
  double km_lim;         // mass estimator limit
  double kq_roll_pitch;  // pitch/roll attitude gain
  double kq_yaw;         // yaw attitude gain
  double kw_roll_pitch;  // attitude rate gain
  double kw_yaw;         // attitude rate gain
  // load gains:
  double kplxy;      // load position xy gain
  double kvlxy;      // load velocity xy gain
  double kplz;       // load position z gain
  double kvlz;       // load velocity z gain
  // TOOD: need to add more load gains?
} Gains_t;

}  // namespace se3_copy_controller

}  // namespace controllers_brubotics

#endif  // CONTROLLERS_BRUBOTICS_SE3_COPY_CONTROLLER_H