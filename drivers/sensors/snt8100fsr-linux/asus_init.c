#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#define CONFIG_I2C_MODULE
#include <linux/i2c.h>
#include "device.h"
#include "memory.h"
#include "serial_bus.h"
#include "main.h"
#include "event.h"
#include "hardware.h"
#include "sysfs.h"
#include "utils.h"
#include "config.h"
#include "debug.h"
#include "sonacomm.h"
#include "workqueue.h"
#include "asus_init.h"

int SNT_SUSPEND_FLAG = 1;
int write_fail_count = 0;
int write_fail_reset_trigger = 1;

//Reset Function
void check_i2c_error(void){
	write_fail_count++;
	if(write_fail_count >= write_fail_reset_trigger){
#ifdef FACTORY_FLAG
#else
	  	ASUSEvtlog("[Grip] read/write fail, count=%d, call reset function\n", write_fail_count);
    	queue_delayed_work(asus_wq, &rst_gpio_wk, msecs_to_jiffies(1000));
#endif
		/* reset write_fail_count in Reset_func */
		//write_fail_count = 0;
	}
}

void snt_set_pinctrl(struct device *dev, char *str)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	//PRINT_INFO("Set_pinctrl start!");
	key_pinctrl = devm_pinctrl_get(dev);
	if(key_pinctrl!=NULL){
		set_state = pinctrl_lookup_state(key_pinctrl, str);
		if(set_state!=NULL){
			//dev_info("pinctrl_lookup_state: set_state=%s", set_state->name));
			ret = pinctrl_select_state(key_pinctrl, set_state);
			if(ret < 0){
				PRINT_ERR("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
			}
			PRINT_INFO("Set_pinctrl done!");
		}
	} else {
		PRINT_ERR("pinctrl_lookup_state: key_pinctrl=NULL!");
	}
}

static struct regulator *reg;

static int grip_snt8155_regulator_init(void)
{
	int ret = 0;
	reg = regulator_get(snt8100fsr_g->dev,"vcc_gripsensor");
    if (IS_ERR_OR_NULL(reg)) {
        ret = PTR_ERR(reg);
        PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
        return ret;
    }
    ret = regulator_set_voltage(reg, 2800000, 2800000);
    if (ret) {
        PRINT_ERR("Failed to set voltage for vcc_gripsensor reg %d\n", ret);
        return -1;
    }
    
    PRINT_INFO("vcc_gripsensor regulator setting init");
    return ret;
}

static int grip_snt8155_regulator_enable(void)
{
    int ret = 0, idx = 0;

    if(IS_ERR_OR_NULL(reg)){
        ret = PTR_ERR(reg);
        PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
        return ret;
    }
    
    ret = regulator_set_load(reg, 10000);
    if(ret < 0){
        PRINT_ERR("Failed to set load for vcc_gripsensor reg %d\n", ret);
        return ret;
    }
    
    ret = regulator_enable(reg);
    if(ret){
        PRINT_ERR("Failed to enable vcc_gripsensor reg %d\n", ret);
        return -1;
    }
    
    for(idx=0; idx<10; idx++){
        if(regulator_is_enabled(reg) > 0){
            PRINT_DEBUG("vcc_gripsensor regulator is enabled(idx=%d)", idx);
            break;
        }
    }
    if(idx >= 10){
        PRINT_ERR("vcc_gripsensor regulator is enabled fail(retry count >= %d)", idx);
        return -1;
    }
    
    PRINT_INFO("Update vcc_psensor to NPM_mode");
    return ret;
}

static int grip_snt8155_regulator_disable(void)
{
	int ret = 0;

    if(IS_ERR_OR_NULL(reg)){
        ret = PTR_ERR(reg);
        PRINT_ERR("Failed to get regulator vcc_gripsensor %d\n", ret);
        return ret;
    }
    
    ret = regulator_set_load(reg, 0);
    if(ret < 0){
        PRINT_ERR("Failed to set load for vcc_gripsensor reg %d\n", ret);
        return ret;
    }
    
    ret = regulator_disable(reg);
    if(ret){
        PRINT_ERR("Failed to enable vincentr reg %d\n", ret);
        return -1;
    }
    
    PRINT_INFO("Update vcc_gripsensor to LPM_mode");
    return ret;
}

static struct grip_asus_struct grip_asus_snt = {
	.grip_regulator_init = grip_snt8155_regulator_init,
	.grip_regulator_enable = grip_snt8155_regulator_enable,
	.grip_regulator_disable = grip_snt8155_regulator_disable,
};

void set_1V2_2V8_pin_func(struct work_struct *work_orig) {
   	uint16_t reg_chidlsb, reg_chidmsb;
	
	PRINT_INFO("1v2 pull up");
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_1V2_ON);
	msleep(5);
	
	PRINT_INFO("2v8 pull up");
	snt8100fsr_g->mgrip_asus_func->grip_regulator_init();	
	snt8100fsr_g->mgrip_asus_func->grip_regulator_enable();	
	
	PRINT_INFO("rst pull up");
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_RST_ON);
	msleep(5);
	
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_LSB, &reg_chidlsb);
	PRINT_INFO("REGISTER_CHIP_ID_LSB = 0x%x", reg_chidlsb);
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_MSB, &reg_chidmsb);
	PRINT_INFO("REGISTER_CHIP_ID_MSB = 0x%x", reg_chidmsb);
	
	//MUTEX_LOCK(&snt8100fsr_g->ap_lock);
    return;
}

void set_1V2_2V8_pin_no_func(void) {
   	uint16_t reg_chidlsb, reg_chidmsb;
	
	PRINT_INFO("1v2 pull up");
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_1V2_ON);
	msleep(5);
	
	PRINT_INFO("2v8 pull up");
	snt8100fsr_g->mgrip_asus_func->grip_regulator_init();	
	snt8100fsr_g->mgrip_asus_func->grip_regulator_enable();	
	
	PRINT_INFO("rst pull up");
	snt_set_pinctrl(snt8100fsr_g->dev, GRIP_RST_ON);
	msleep(5);
	
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_LSB, &reg_chidlsb);
	PRINT_INFO("REGISTER_CHIP_ID_LSB = 0x%x", reg_chidlsb);
	read_register(snt8100fsr_g, REGISTER_CHIP_ID_MSB, &reg_chidmsb);
	PRINT_INFO("REGISTER_CHIP_ID_MSB = 0x%x", reg_chidmsb);
	
	//MUTEX_LOCK(&snt8100fsr_g->ap_lock);
    return;
}

void asus_init_probe(void){
	int ret;
	//struct delayed_work en_pwr_wk;
	struct Grip_DPC_status *Grip_DPC_status_t;
	struct grip_status *grip_state_t;

    //Initialization for main i2c device only, should be put after this line
    Grip_DPC_status_t = memory_allocate(sizeof(*Grip_DPC_status_t),
                                 GFP_KERNEL);
    Grip_DPC_status_t->Condition = 0xa710;
	Grip_DPC_status_t->High = 0x14;
    Grip_DPC_status_t->Low = 0x5;
    Grip_DPC_status_g = Grip_DPC_status_t;
	
    grip_state_t = memory_allocate(sizeof(*grip_state_t),
                                 GFP_KERNEL);
	
	/* fw status default value */
	snt8100fsr_g->grip_fw_loading_status = false;
	snt8100fsr_g->fw_sec_source = false;
	snt8100fsr_g->fw_info_check = false;
	
    Grip_DPC_status_g = Grip_DPC_status_t;
    memset(grip_state_t, -1, sizeof(*grip_state_t));
    grip_status_g = grip_state_t;

	snt8100fsr_g->mgrip_asus_func = &grip_asus_snt;
	
    asus_wq = create_workqueue(asus_grip_queue);
    if (!asus_wq) {
        PRINT_CRIT("Unable to create_workqueue(%s)", asus_grip_queue);
        return;
    }
    INIT_DELAYED_WORK(&check_stuck_wake, check_stuck_semaphore);
    INIT_DELAYED_WORK(&rst_recovery_wk, grip_dump_status_func);
    INIT_DELAYED_WORK(&rst_gpio_wk, Reset_Func);
    wake_lock_init(&(snt8100fsr_g->snt_wakelock), WAKE_LOCK_SUSPEND, "snt_wakelock"); 


    /* Feed 1v2 and 2v8 to the chip */
    //if (of_property_read_bool(np, "grip_gpio12")){

	
	//INIT_DELAYED_WORK(&en_pwr_wk, set_1V2_2V8_pin_func);
	PRINT_INFO("WQ: call set_rst_pin_func");
	//workqueue_queue_work(&en_pwr_wk, 0);
	set_1V2_2V8_pin_no_func();

    /* Clay ioctl +++*/
    ret = sntSensor_miscRegister();
    if (ret < 0) {
		PRINT_INFO("creat misc fail");
    }
	

    create_Grip_en_proc_file();
    create_Grip_frame_proc_file();
    create_Grip_raw_en_proc_file();
	
    /* Tap proc */
	create_Grip_Tap_En_proc_file();
	create_Grip_Tap_Force_proc_file();
	create_Grip_Tap_min_pos_proc_file();
	create_Grip_Tap_max_pos_proc_file();
	create_Grip_Tap_slope_window_proc_file();
	create_Grip_Tap_slope_tap_force_proc_file();
	create_Grip_Tap_slope_release_force_proc_file();
	create_Grip_Tap_delta_tap_force_proc_file();
	create_Grip_Tap_delta_release_force_proc_file();
	create_Grip_Tap_vib_en_proc_file();

	create_Grip_Squeeze_en_proc_file();
	create_Grip_Squeeze_force_proc_file();
	create_Grip_Squeeze_short_dur_proc_file();
	create_Grip_Squeeze_long_dur_proc_file();
	create_Grip_Squeeze_drop_rate_proc_file();
	create_Grip_Squeeze_drop_total_proc_file();
	create_Grip_Squeeze_up_rate_proc_file();
	create_Grip_Squeeze_up_total_proc_file();
	
	create_Grip_Slide_en_proc_file();
	create_Grip_Slide_dist_proc_file();
	create_Grip_Slide_2nd_dist_proc_file();
	create_Grip_Slide_force_proc_file();
	create_Grip_Slide_min_pos_proc_file();
	create_Grip_Slide_max_pos_proc_file();
	create_Grip_Slide_vib_en_proc_file();
	
	create_Grip_Swipe_en_proc_file();
	create_Grip_Swipe_velocity_proc_file();
	create_Grip_Swipe_len_proc_file();
	create_Grip_Swipe_min_pos_proc_file();
	create_Grip_Swipe_max_pos_proc_file();
	
    create_Grip_Tap1_En_proc_file();
    create_Grip_Tap2_En_proc_file();
    create_Grip_Tap_Sense_En_proc_file();
    create_Grip_Tap1_Force_proc_file();
    create_Grip_Tap2_Force_proc_file();
    create_Grip_Tap1_Vibtator_enable_proc_file();
    create_Grip_Tap2_Vibtator_enable_proc_file();
    create_Grip_Tap1_Rest_enable_proc_file();
    create_Grip_Tap2_Rest_enable_proc_file();
    create_Grip_Tap1_MIN_Position_proc_file();
    create_Grip_Tap2_MIN_Position_proc_file();
    create_Grip_Tap1_MAX_Position_proc_file();
    create_Grip_Tap2_MAX_Position_proc_file();
    create_Grip_Tap1_slope_window_proc_file();
    create_Grip_Tap2_slope_window_proc_file();
    create_Grip_Tap1_slope_tap_force_proc_file();
    create_Grip_Tap2_slope_tap_force_proc_file();
    create_Grip_Tap1_slope_release_force_proc_file();
    create_Grip_Tap2_slope_release_force_proc_file();
    create_Grip_Tap1_delta_tap_force_proc_file();
    create_Grip_Tap2_delta_tap_force_proc_file();
    create_Grip_Tap1_delta_release_force_proc_file();
    create_Grip_Tap2_delta_release_force_proc_file();
	
    /* Squeeze proc */
    create_Grip_Squeeze1_En_proc_file();
    create_Grip_Squeeze1_Force_proc_file();
    create_Grip_Squeeze1_short_dur_proc_file();
    create_Grip_Squeeze1_long_dur_proc_file();
    create_Grip_Squeeze1_up_rate_proc_file();
    create_Grip_Squeeze1_up_total_proc_file();
    create_Grip_Squeeze1_drop_rate_proc_file();
    create_Grip_Squeeze1_drop_total_proc_file();
	
    create_Grip_Squeeze2_En_proc_file();
    create_Grip_Squeeze2_Force_proc_file();
    create_Grip_Squeeze2_short_dur_proc_file();
    create_Grip_Squeeze2_long_dur_proc_file();
    create_Grip_Squeeze2_up_rate_proc_file();
    create_Grip_Squeeze2_up_total_proc_file();
    create_Grip_Squeeze2_drop_rate_proc_file();
    create_Grip_Squeeze2_drop_total_proc_file();
	
    /* Swipe proc */
    create_Grip_Swipe1_En_proc_file();
    create_Grip_Swipe2_En_proc_file();
    create_Grip_Swipe1_Velocity_proc_file();
    create_Grip_Swipe2_Velocity_proc_file();
    create_Grip_Swipe1_Len_proc_file();
    create_Grip_Swipe2_Len_proc_file();
	
    /* Slide proc */
    create_Grip_Slide1_En_proc_file();
    create_Grip_Slide2_En_proc_file();
    create_Grip_Slide1_Distance_proc_file();
    create_Grip_Slide2_Distance_proc_file();
    create_Grip_Slide1_force_proc_file();
    create_Grip_Slide2_force_proc_file();
    create_Grip_Slide1_Vibtator_enable_proc_file();
    create_Grip_Slide2_Vibtator_enable_proc_file();

    /******* Dynamic Loading FW ********/
    create_Grip_FW_VER_proc_file();
    create_Grip_FW_RESULT_proc_file();
    create_Grip_set_power_proc_file();

    /* Squeeze Factor */
    create_Grip_SQ_Bar0_factory_proc_file();
    create_Grip_SQ_Bar1_factory_proc_file();
    create_Grip_SQ_Bar2_factory_proc_file();

    /* Factory requirment */
    create_Grip_I2c_Check_proc_file();
    create_Grip_FPC_Check_proc_file();
    create_Grip_Calibration_raw_data_proc_file();
    create_Grip_Disable_WakeLock_proc_file();
    create_Grip_Apply_GoldenK_proc_file();
    create_Grip_ReadK_proc_file();

}

