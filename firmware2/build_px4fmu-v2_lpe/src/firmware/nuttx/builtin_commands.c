/* builtin command list - automatically generated, do not edit */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>
#include <nuttx/config.h>
extern int adc_main(int argc, char *argv[]);
extern int tone_alarm_main(int argc, char *argv[]);
extern int fmu_main(int argc, char *argv[]);
extern int px4io_main(int argc, char *argv[]);
extern int rgbled_main(int argc, char *argv[]);
extern int mpu6000_main(int argc, char *argv[]);
extern int mpu9250_main(int argc, char *argv[]);
extern int lsm303d_main(int argc, char *argv[]);
extern int l3gd20_main(int argc, char *argv[]);
extern int hmc5883_main(int argc, char *argv[]);
extern int ms5611_main(int argc, char *argv[]);
extern int srf02_main(int argc, char *argv[]);
extern int sf0x_main(int argc, char *argv[]);
extern int ll40ls_main(int argc, char *argv[]);
extern int trone_main(int argc, char *argv[]);
extern int gps_main(int argc, char *argv[]);
extern int pwm_out_sim_main(int argc, char *argv[]);
extern int blinkm_main(int argc, char *argv[]);
extern int ets_airspeed_main(int argc, char *argv[]);
extern int meas_airspeed_main(int argc, char *argv[]);
extern int frsky_telemetry_main(int argc, char *argv[]);
extern int sensors_main(int argc, char *argv[]);
extern int px4flow_main(int argc, char *argv[]);
extern int oreoled_main(int argc, char *argv[]);
extern int gimbal_main(int argc, char *argv[]);
extern int pwm_input_main(int argc, char *argv[]);
extern int camera_trigger_main(int argc, char *argv[]);
extern int bst_main(int argc, char *argv[]);
extern int snapdragon_rc_pwm_main(int argc, char *argv[]);
extern int bl_update_main(int argc, char *argv[]);
extern int mixer_main(int argc, char *argv[]);
extern int param_main(int argc, char *argv[]);
extern int perf_main(int argc, char *argv[]);
extern int pwm_main(int argc, char *argv[]);
extern int esc_calib_main(int argc, char *argv[]);
extern int reboot_main(int argc, char *argv[]);
extern int top_main(int argc, char *argv[]);
extern int config_main(int argc, char *argv[]);
extern int nshterm_main(int argc, char *argv[]);
extern int mtd_main(int argc, char *argv[]);
extern int dumpfile_main(int argc, char *argv[]);
extern int ver_main(int argc, char *argv[]);
extern int commander_main(int argc, char *argv[]);
extern int navigator_main(int argc, char *argv[]);
extern int mavlink_main(int argc, char *argv[]);
extern int gpio_led_main(int argc, char *argv[]);
extern int uavcan_main(int argc, char *argv[]);
extern int land_detector_main(int argc, char *argv[]);
extern int fw_pos_control_l1_main(int argc, char *argv[]);
extern int fw_att_control_main(int argc, char *argv[]);
extern int mc_att_control_main(int argc, char *argv[]);
extern int mc_pos_control_main(int argc, char *argv[]);
extern int vtol_att_control_main(int argc, char *argv[]);
extern int sdlog2_main(int argc, char *argv[]);
extern int uorb_main(int argc, char *argv[]);
extern int dataman_main(int argc, char *argv[]);
extern int rover_steering_control_main(int argc, char *argv[]);
extern int local_position_estimator_main(int argc, char *argv[]);
extern int attitude_estimator_q_main(int argc, char *argv[]);
extern int serdis_main(int argc, char *argv[]);
extern int sercon_main(int argc, char *argv[]);

const struct builtin_s g_builtins[] = {
	{"adc", SCHED_PRIORITY_DEFAULT, 1024, adc_main},
	{"tone_alarm", SCHED_PRIORITY_DEFAULT, 1024, tone_alarm_main},
	{"fmu", SCHED_PRIORITY_DEFAULT, 1200, fmu_main},
	{"px4io", SCHED_PRIORITY_DEFAULT, 1800, px4io_main},
	{"rgbled", SCHED_PRIORITY_DEFAULT, 1024, rgbled_main},
	{"mpu6000", SCHED_PRIORITY_DEFAULT, 1200, mpu6000_main},
	{"mpu9250", SCHED_PRIORITY_DEFAULT, 1200, mpu9250_main},
	{"lsm303d", SCHED_PRIORITY_DEFAULT, 1200, lsm303d_main},
	{"l3gd20", SCHED_PRIORITY_DEFAULT, 1200, l3gd20_main},
	{"hmc5883", SCHED_PRIORITY_DEFAULT, 1200, hmc5883_main},
	{"ms5611", SCHED_PRIORITY_DEFAULT, 1024, ms5611_main},
	{"srf02", SCHED_PRIORITY_DEFAULT, 1024, srf02_main},
	{"sf0x", SCHED_PRIORITY_DEFAULT, 1024, sf0x_main},
	{"ll40ls", SCHED_PRIORITY_DEFAULT, 1024, ll40ls_main},
	{"trone", SCHED_PRIORITY_DEFAULT, 1200, trone_main},
	{"gps", SCHED_PRIORITY_DEFAULT, 1200, gps_main},
	{"pwm_out_sim", SCHED_PRIORITY_DEFAULT, 1200, pwm_out_sim_main},
	{"blinkm", SCHED_PRIORITY_DEFAULT, 1024, blinkm_main},
	{"ets_airspeed", SCHED_PRIORITY_DEFAULT, 1200, ets_airspeed_main},
	{"meas_airspeed", SCHED_PRIORITY_DEFAULT, 1200, meas_airspeed_main},
	{"frsky_telemetry", SCHED_PRIORITY_DEFAULT, 1200, frsky_telemetry_main},
	{"sensors", SCHED_PRIORITY_MAX-5, 1200, sensors_main},
	{"px4flow", SCHED_PRIORITY_DEFAULT, 1200, px4flow_main},
	{"oreoled", SCHED_PRIORITY_DEFAULT, 1024, oreoled_main},
	{"gimbal", SCHED_PRIORITY_DEFAULT, 1024, gimbal_main},
	{"pwm_input", SCHED_PRIORITY_DEFAULT, 1024, pwm_input_main},
	{"camera_trigger", SCHED_PRIORITY_DEFAULT, 1200, camera_trigger_main},
	{"bst", SCHED_PRIORITY_DEFAULT, 1200, bst_main},
	{"snapdragon_rc_pwm", SCHED_PRIORITY_DEFAULT, 1024, snapdragon_rc_pwm_main},
	{"bl_update", SCHED_PRIORITY_DEFAULT, 4096, bl_update_main},
	{"mixer", SCHED_PRIORITY_DEFAULT, 4096, mixer_main},
	{"param", SCHED_PRIORITY_DEFAULT, 2500, param_main},
	{"perf", SCHED_PRIORITY_DEFAULT, 1800, perf_main},
	{"pwm", SCHED_PRIORITY_DEFAULT, 2400, pwm_main},
	{"esc_calib", SCHED_PRIORITY_DEFAULT, 4096, esc_calib_main},
	{"reboot", SCHED_PRIORITY_DEFAULT, 800, reboot_main},
	{"top", SCHED_PRIORITY_DEFAULT, 1700, top_main},
	{"config", SCHED_PRIORITY_DEFAULT, 4096, config_main},
	{"nshterm", SCHED_PRIORITY_DEFAULT-30, 1500, nshterm_main},
	{"mtd", SCHED_PRIORITY_DEFAULT, 1024, mtd_main},
	{"dumpfile", SCHED_PRIORITY_DEFAULT, 1024, dumpfile_main},
	{"ver", SCHED_PRIORITY_DEFAULT, 1024, ver_main},
	{"commander", SCHED_PRIORITY_DEFAULT, 4096, commander_main},
	{"navigator", SCHED_PRIORITY_DEFAULT, 1200, navigator_main},
	{"mavlink", SCHED_PRIORITY_DEFAULT, 1200, mavlink_main},
	{"gpio_led", SCHED_PRIORITY_DEFAULT, 1024, gpio_led_main},
	{"uavcan", SCHED_PRIORITY_DEFAULT, 3200, uavcan_main},
	{"land_detector", SCHED_PRIORITY_DEFAULT, 1200, land_detector_main},
	{"fw_pos_control_l1", SCHED_PRIORITY_DEFAULT, 1200, fw_pos_control_l1_main},
	{"fw_att_control", SCHED_PRIORITY_DEFAULT, 1200, fw_att_control_main},
	{"mc_att_control", SCHED_PRIORITY_DEFAULT, 1200, mc_att_control_main},
	{"mc_pos_control", SCHED_PRIORITY_DEFAULT, 1200, mc_pos_control_main},
	{"vtol_att_control", SCHED_PRIORITY_DEFAULT, 1024, vtol_att_control_main},
	{"sdlog2", SCHED_PRIORITY_MAX-30, 1200, sdlog2_main},
	{"uorb", SCHED_PRIORITY_DEFAULT, 2048, uorb_main},
	{"dataman", SCHED_PRIORITY_DEFAULT, 1200, dataman_main},
	{"rover_steering_control", SCHED_PRIORITY_DEFAULT, 1200, rover_steering_control_main},
	{"local_position_estimator", SCHED_PRIORITY_DEFAULT, 5700, local_position_estimator_main},
	{"attitude_estimator_q", SCHED_PRIORITY_DEFAULT, 1200, attitude_estimator_q_main},
	{"serdis", SCHED_PRIORITY_DEFAULT, 2048, serdis_main},
	{"sercon", SCHED_PRIORITY_DEFAULT, 2048, sercon_main},

	{NULL, 0, 0, NULL}
};
const int g_builtin_count = 61;
