#include <linux/proc_fs.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
//#include <linux/wakelock.h>
#include "grip_Wakelock.h"

extern int sntSensor_miscRegister(void);

extern void create_Grip_I2c_Check_proc_file(void);
extern void create_Grip_FPC_Check_proc_file(void);
extern void create_Grip_Calibration_raw_data_proc_file(void);
extern void create_Grip_Disable_WakeLock_proc_file(void);

//Enable interface
extern void create_Grip_frame_proc_file(void);
extern void create_Grip_raw_en_proc_file(void);
extern void create_Grip_en_proc_file(void);
	
//Gesture proc
extern void create_Grip_Tap_En_proc_file(void);
extern void create_Grip_Tap_Force_proc_file(void);
extern void create_Grip_Tap_min_pos_proc_file(void);
extern void create_Grip_Tap_max_pos_proc_file(void);
extern void create_Grip_Tap_slope_window_proc_file(void);
extern void create_Grip_Tap_slope_tap_force_proc_file(void);
extern void create_Grip_Tap_slope_release_force_proc_file(void);
extern void create_Grip_Tap_delta_tap_force_proc_file(void);
extern void create_Grip_Tap_delta_release_force_proc_file(void);
extern void create_Grip_Tap_vib_en_proc_file(void);

extern void create_Grip_Squeeze_en_proc_file(void);
extern void create_Grip_Squeeze_force_proc_file(void);
extern void create_Grip_Squeeze_force_proc_file(void);
extern void create_Grip_Squeeze_short_dur_proc_file(void);
extern void create_Grip_Squeeze_long_dur_proc_file(void);
extern void create_Grip_Squeeze_drop_rate_proc_file(void);
extern void create_Grip_Squeeze_drop_total_proc_file(void);
extern void create_Grip_Squeeze_up_rate_proc_file(void);
extern void create_Grip_Squeeze_up_total_proc_file(void);

extern void create_Grip_Slide_en_proc_file(void);
extern void create_Grip_Slide_dist_proc_file(void);
extern void create_Grip_Slide_2nd_dist_proc_file(void);
extern void create_Grip_Slide_force_proc_file(void);
extern void create_Grip_Slide_min_pos_proc_file(void);
extern void create_Grip_Slide_max_pos_proc_file(void);
extern void create_Grip_Slide_vib_en_proc_file(void);

extern void create_Grip_Swipe_en_proc_file(void);
extern void create_Grip_Swipe_velocity_proc_file(void);
extern void create_Grip_Swipe_len_proc_file(void);
extern void create_Grip_Swipe_min_pos_proc_file(void);
extern void create_Grip_Swipe_max_pos_proc_file(void);


extern void create_Grip_Tap1_En_proc_file(void);
extern void create_Grip_Tap2_En_proc_file(void);
extern void create_Grip_Tap3_En_proc_file(void);
extern void create_Grip_Tap_Sense_En_proc_file(void);
extern void create_Grip_Tap1_Force_proc_file(void);
extern void create_Grip_Tap2_Force_proc_file(void);
extern void create_Grip_Tap3_Force_proc_file(void);
extern void create_Grip_Tap1_Vibtator_enable_proc_file(void);
extern void create_Grip_Tap2_Vibtator_enable_proc_file(void);
extern void create_Grip_Tap1_Rest_enable_proc_file(void);
extern void create_Grip_Tap2_Rest_enable_proc_file(void);
extern void create_Grip_Tap1_MIN_Position_proc_file(void);
extern void create_Grip_Tap2_MIN_Position_proc_file(void);
extern void create_Grip_Tap1_MAX_Position_proc_file(void);
extern void create_Grip_Tap2_MAX_Position_proc_file(void);
extern void create_Grip_Tap1_slope_window_proc_file(void);
extern void create_Grip_Tap2_slope_window_proc_file(void);
extern void create_Grip_Tap1_slope_tap_force_proc_file(void);
extern void create_Grip_Tap2_slope_tap_force_proc_file(void);
extern void create_Grip_Tap1_slope_release_force_proc_file(void);
extern void create_Grip_Tap2_slope_release_force_proc_file(void);
extern void create_Grip_Tap1_delta_tap_force_proc_file(void);
extern void create_Grip_Tap2_delta_tap_force_proc_file(void);
extern void create_Grip_Tap1_delta_release_force_proc_file(void);
extern void create_Grip_Tap2_delta_release_force_proc_file(void);

extern void create_Grip_Squeeze1_En_proc_file(void);
extern void create_Grip_Squeeze1_Force_proc_file(void);
extern void create_Grip_Squeeze1_short_dur_proc_file(void);
extern void create_Grip_Squeeze1_long_dur_proc_file(void);
extern void create_Grip_Squeeze1_up_rate_proc_file(void);
extern void create_Grip_Squeeze1_up_total_proc_file(void);
extern void create_Grip_Squeeze1_drop_rate_proc_file(void);
extern void create_Grip_Squeeze1_drop_total_proc_file(void);
extern void create_Grip_Squeeze2_En_proc_file(void);
extern void create_Grip_Squeeze2_Force_proc_file(void);
extern void create_Grip_Squeeze2_short_dur_proc_file(void);
extern void create_Grip_Squeeze2_long_dur_proc_file(void);
extern void create_Grip_Squeeze2_up_rate_proc_file(void);
extern void create_Grip_Squeeze2_up_total_proc_file(void);
extern void create_Grip_Squeeze2_drop_rate_proc_file(void);
extern void create_Grip_Squeeze2_drop_total_proc_file(void);

extern void create_Grip_Slide1_En_proc_file(void);
extern void create_Grip_Slide2_En_proc_file(void);
extern void create_Grip_Slide1_Distance_proc_file(void);
extern void create_Grip_Slide2_Distance_proc_file(void);
extern void create_Grip_Slide1_force_proc_file(void);
extern void create_Grip_Slide2_force_proc_file(void);
extern void create_Grip_Slide1_Vibtator_enable_proc_file(void);
extern void create_Grip_Slide2_Vibtator_enable_proc_file(void);
extern void create_Grip_Swipe1_En_proc_file(void);
extern void create_Grip_Swipe2_En_proc_file(void);
extern void create_Grip_Swipe1_Len_proc_file(void);
extern void create_Grip_Swipe2_Len_proc_file(void);
extern void create_Grip_Swipe1_Velocity_proc_file(void);
extern void create_Grip_Swipe2_Velocity_proc_file(void);



//Function: DPC wake from low power mode
extern void Wait_Wake_For_RegW(void);
extern void DPC_write_func(int flag);

// Gesture enable func
extern void grip_raw_enable_func(int val);
extern void grip_enable_func_noLock(int val);
extern void grip_tap1_enable_func(int val);
extern void grip_tap2_enable_func(int val);
extern void grip_tap3_enable_func(int val);
extern void grip_tap1_force_func(int val);
extern void grip_tap2_force_func(int val);
extern void grip_tap3_force_func(int val);
extern void grip_tap_sense_enable_func(int val);
extern void grip_tap1_vibrator_enable_func(int val);
extern void grip_tap2_vibrator_enable_func(int val);
extern void grip_tap1_finger_reseting_enable_func(int val);
extern void grip_tap2_finger_reseting_enable_func(int val);
extern void grip_tap1_min_position_func(int val);
extern void grip_tap2_min_position_func(int val);
extern void grip_tap3_min_position_func(int val);
extern void grip_tap1_max_position_func(int val);
extern void grip_tap2_max_position_func(int val);
extern void grip_tap3_max_position_func(int val);

//Gesture Threshold func
extern void grip_squeeze1_enable_func(int val);
extern void grip_squeeze1_force_func(int val);
extern void grip_squeeze1_long_dur_func(int val);
extern void grip_squeeze1_short_dur_func(int val);
extern void grip_squeeze1_up_rate_func(int val);
extern void grip_squeeze1_up_total_func(int val);
extern void grip_squeeze1_drop_rate_func(int val);
extern void grip_squeeze1_drop_total_func(int val);
extern void grip_squeeze2_enable_func(int val);
extern void grip_squeeze2_force_func(int val);
extern void grip_squeeze2_long_dur_func(int val);
extern void grip_squeeze2_short_dur_func(int val);
extern void grip_squeeze2_up_rate_func(int val);
extern void grip_squeeze2_up_total_func(int val);
extern void grip_squeeze2_drop_rate_func(int val);
extern void grip_squeeze2_drop_total_func(int val);

extern void grip_slide1_enable_func(int val);
extern void grip_slide2_enable_func(int val);
extern void grip_slide1_dist_func(int val);
extern void grip_slide2_dist_func(int val);
extern void grip_slide1_force_func(int val);
extern void grip_slide2_force_func(int val);

extern void grip_swipe1_enable_func(int val);
extern void grip_swipe2_enable_func(int val);
extern void grip_swipe1_velocity_func(int val);
extern void grip_swipe2_velocity_func(int val);
extern void grip_swipe1_len_func(int val);
extern void grip_swipe2_len_func(int val);

#ifdef DYNAMIC_PWR_CTL
extern int snt_activity_request(void);
#endif

extern int Health_Check_Enable(int en);
extern void Into_DeepSleep_fun(void);
/******* Dynamic Loading FW ********/
extern int fw_version;
extern void create_Grip_FW_RESULT_proc_file(void);
extern void create_Grip_FW_VER_proc_file(void);
extern void create_Grip_set_power_proc_file(void);
extern void create_Grip_SQ_Bar0_factory_proc_file(void);
extern void create_Grip_SQ_Bar1_factory_proc_file(void);
extern void create_Grip_SQ_Bar2_factory_proc_file(void);
extern void create_Grip_Apply_GoldenK_proc_file(void);
extern void create_Grip_ReadK_proc_file(void);

/* used to record health check value */
extern uint16_t FPC_value;

extern uint16_t Grip_B0_F_value;
extern uint16_t Grip_B1_F_value;
extern uint16_t Grip_B2_F_value;
extern bool G_Skip_Sq1_Long;
extern bool G_Skip_Sq2_Long;
extern enum DEVICE_HWID g_ASUS_hwID;


/* Workaround for stucked semaphore */
extern struct delayed_work check_stuck_wake;
extern void check_stuck_semaphore(struct work_struct *work);


extern struct delayed_work rst_recovery_wk;
extern struct delayed_work rst_gpio_wk;
extern void Reset_Func(struct work_struct *work);
extern void grip_dump_status_func(struct work_struct *work);
extern struct workqueue_struct *asus_wq;


extern void set_sq_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_tap_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_slide_gesture(uint16_t slide_id, uint16_t reg_val, int index);
extern void set_swipe_gesture(uint16_t slide_id, uint16_t reg_val, int index);

extern void get_sq_gesture(uint16_t tap_id, uint16_t reg_val, int index, int len);
/* Enable/disable Grip Sensor Power 1V2_2V8 */
extern void Power_Control(int en); 

extern void Grip_Driver_IRQ_EN(bool flag);
extern void Grip_Chip_IRQ_EN(bool flag);

extern int grip_game_gesture_status(void);

extern enum DEVICE_PROJID g_ASUS_prjID;
extern enum DEVICE_HWID g_ASUS_hwID;
