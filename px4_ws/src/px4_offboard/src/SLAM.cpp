#include "px4_offboard/include.h"
#include "px4_offboard/CtrlPx4.h"
#include "CtrlPx4.cpp"

// enum {takeoff, calibration, traverse, tracking, land};

int main(int argv, char **argc) {
  ros::init(argv, argc, "px4_controller");
  CtrlPx4 controller;
  ros::Rate loop_rate(200);

  while (ros::ok()) {

// #ifdef AUTO_FLIGHT
//   	if (controller.getTakeoffSignal())
//   	{
// 	  	switch(flightMode)
// 	  	{
// 	  		case takeoff:
// 	  			if(controller.takeoff(1.5 ,2))
// 	  				flightMode = calibration;
// 	  		break;

// 	  		case calibration:
// 	  			controller.land(2);

// 	  		break;

// 	  		case traverse:
// 	  		break;

// 	  		case tracking:
// 	  		break;

// 	  		case land:
// 	  		break;
// 	  	}
//  		controller.commandUpdate();
// 	}

// #else
    controller.commandUpdate();
// #endif

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
