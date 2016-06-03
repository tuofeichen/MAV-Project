/* builtin command list - automatically generated, do not edit */
#include <string>
#include <map>
#include <stdio.h>

#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_log.h>
#include <stdlib.h>

using namespace std;

extern void px4_show_devices(void);

extern "C" {
extern int pwm_out_sim_main(int argc, char *argv[]);
extern int adcsim_main(int argc, char *argv[]);
extern int gpssim_main(int argc, char *argv[]);
extern int tone_alarm_main(int argc, char *argv[]);
extern int accelsim_main(int argc, char *argv[]);
extern int measairspeedsim_main(int argc, char *argv[]);
extern int barosim_main(int argc, char *argv[]);
extern int gyrosim_main(int argc, char *argv[]);
extern int rgbledsim_main(int argc, char *argv[]);
extern int param_main(int argc, char *argv[]);
extern int mixer_main(int argc, char *argv[]);
extern int ver_main(int argc, char *argv[]);
extern int esc_calib_main(int argc, char *argv[]);
extern int reboot_main(int argc, char *argv[]);
extern int listener_main(int argc, char *argv[]);
extern int perf_main(int argc, char *argv[]);
extern int uorb_main(int argc, char *argv[]);
extern int sensors_main(int argc, char *argv[]);
extern int simulator_main(int argc, char *argv[]);
extern int mavlink_main(int argc, char *argv[]);
extern int attitude_estimator_ekf_main(int argc, char *argv[]);
extern int attitude_estimator_q_main(int argc, char *argv[]);
extern int ekf2_main(int argc, char *argv[]);
extern int local_position_estimator_main(int argc, char *argv[]);
extern int ekf_att_pos_estimator_main(int argc, char *argv[]);
extern int position_estimator_inav_main(int argc, char *argv[]);
extern int navigator_main(int argc, char *argv[]);
extern int vtol_att_control_main(int argc, char *argv[]);
extern int mc_pos_control_main(int argc, char *argv[]);
extern int mc_att_control_main(int argc, char *argv[]);
extern int mc_pos_control_m_main(int argc, char *argv[]);
extern int mc_att_control_m_main(int argc, char *argv[]);
extern int land_detector_main(int argc, char *argv[]);
extern int fw_att_control_main(int argc, char *argv[]);
extern int fw_pos_control_l1_main(int argc, char *argv[]);
extern int dataman_main(int argc, char *argv[]);
extern int sdlog2_main(int argc, char *argv[]);
extern int commander_main(int argc, char *argv[]);
extern int controllib_test_main(int argc, char *argv[]);
extern int px4_simple_app_main(int argc, char *argv[]);

static int shutdown_main(int argc, char *argv[]);
static int list_tasks_main(int argc, char *argv[]);
static int list_files_main(int argc, char *argv[]);
static int list_devices_main(int argc, char *argv[]);
static int list_topics_main(int argc, char *argv[]);
static int sleep_main(int argc, char *argv[]);
}

static map<string,px4_main_t> app_map(void)
{
        static map<string,px4_main_t> apps;

	apps["pwm_out_sim"] = pwm_out_sim_main;
	apps["adcsim"] = adcsim_main;
	apps["gpssim"] = gpssim_main;
	apps["tone_alarm"] = tone_alarm_main;
	apps["accelsim"] = accelsim_main;
	apps["measairspeedsim"] = measairspeedsim_main;
	apps["barosim"] = barosim_main;
	apps["gyrosim"] = gyrosim_main;
	apps["rgbledsim"] = rgbledsim_main;
	apps["param"] = param_main;
	apps["mixer"] = mixer_main;
	apps["ver"] = ver_main;
	apps["esc_calib"] = esc_calib_main;
	apps["reboot"] = reboot_main;
	apps["listener"] = listener_main;
	apps["perf"] = perf_main;
	apps["uorb"] = uorb_main;
	apps["sensors"] = sensors_main;
	apps["simulator"] = simulator_main;
	apps["mavlink"] = mavlink_main;
	apps["attitude_estimator_ekf"] = attitude_estimator_ekf_main;
	apps["attitude_estimator_q"] = attitude_estimator_q_main;
	apps["ekf2"] = ekf2_main;
	apps["local_position_estimator"] = local_position_estimator_main;
	apps["ekf_att_pos_estimator"] = ekf_att_pos_estimator_main;
	apps["position_estimator_inav"] = position_estimator_inav_main;
	apps["navigator"] = navigator_main;
	apps["vtol_att_control"] = vtol_att_control_main;
	apps["mc_pos_control"] = mc_pos_control_main;
	apps["mc_att_control"] = mc_att_control_main;
	apps["mc_pos_control_m"] = mc_pos_control_m_main;
	apps["mc_att_control_m"] = mc_att_control_m_main;
	apps["land_detector"] = land_detector_main;
	apps["fw_att_control"] = fw_att_control_main;
	apps["fw_pos_control_l1"] = fw_pos_control_l1_main;
	apps["dataman"] = dataman_main;
	apps["sdlog2"] = sdlog2_main;
	apps["commander"] = commander_main;
	apps["controllib_test"] = controllib_test_main;
	apps["px4_simple_app"] = px4_simple_app_main;

	apps["shutdown"] = shutdown_main;
	apps["list_tasks"] = list_tasks_main;
	apps["list_files"] = list_files_main;
	apps["list_devices"] = list_devices_main;
	apps["list_topics"] = list_topics_main;
	apps["sleep"] = sleep_main;

	return apps;
}

map<string,px4_main_t> apps = app_map();

static void list_builtins(void)
{
	cout << "Builtin Commands:" << endl;
	for (map<string,px4_main_t>::iterator it=apps.begin(); it!=apps.end(); ++it)
		cout << '\t' << it->first << endl;
}

static int shutdown_main(int argc, char *argv[])
{
	printf("Shutting down\n");
	exit(0);
}

static int list_tasks_main(int argc, char *argv[])
{
	px4_show_tasks();
	return 0;
}

static int list_devices_main(int argc, char *argv[])
{
	px4_show_devices();
	return 0;
}

static int list_topics_main(int argc, char *argv[])
{
	px4_show_topics();
	return 0;
}

static int list_files_main(int argc, char *argv[])
{
	px4_show_files();
	return 0;
}

static int sleep_main(int argc, char *argv[])
{
	if (argc != 2) {
		cout << "Usage: sleep <seconds>" << endl;
		return 1;
	}
	sleep(atoi(argv[1]));
	return 0;
}

