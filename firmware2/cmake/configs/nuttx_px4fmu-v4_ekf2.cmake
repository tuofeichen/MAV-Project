include(cmake/configs/nuttx_px4fmu-v4_default.cmake)

list(REMOVE_ITEM config_module_list
	modules/local_position_estimator
	modules/attitude_estimator_q
	)

list(APPEND config_module_list
	modules/ekf2
	)
