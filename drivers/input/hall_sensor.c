#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>


//<ASUS-danielchan201500513>>>>>>>>>+
#include <linux/kernel.h>
//<ASUS-danielchan20150513<<<<<<<<<<+

//<ASUS-danielchan201500519>>>>>>>>>+
#include <linux/proc_fs.h>
//<ASUS-danielchan20150519<<<<<<<<<<+

#include <linux/of_gpio.h>
//#include <asm/intel-mid.h>

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef HALL_DEBUG
	#define dbg(fmt, args...) printk("[%s] "fmt,DRIVER_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk(KERN_INFO"[%s] "fmt,DRIVER_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,DRIVER_NAME,##args)

/*****************************/
/* Hall Sensor Configuration */
/****************************/
#define DRIVER_NAME 		"hall_sensor"
#define GPIO_NAME 		"hall_det#"
#define IRQ_Name			"ASUS_Hall_Sensor-irq"
#define INT_NAME			"HallSensor_INT"
#define KERNEL_OBJECT	"hall_sensor_kobject"
#define HALLSEN_ID        924

#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
#define DEBUG_LOG 1
#else
#define DEBUG_LOG 0
#endif

/**************************/
/* Driver Data Structure */
/*************************/
static struct hall_sensor_str { 	
	int status;	
	int enable; 
	spinlock_t mHallSensorLock;
	struct wakeup_source wake_src;
	struct input_dev *hall_indev;	
 	struct delayed_work hall_sensor_work;
	//<ASUS-danielchan201500528>>>>>>>>>+
    int irq_trigger;
	int hardcode;
    int debounce;
    int sleep;
	//<ASUS-danielchan20150528<<<<<<<<<<+
}* hall_sensor_dev;

/*******************************/
/* Hall Sensor Global Variables */
/******************************/
static int 							ASUS_HALL_SENSOR_GPIO;
static int 							ASUS_HALL_SENSOR_IRQ;
static struct workqueue_struct 	*hall_sensor_wq;

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	dbg("[ISR] hall_sensor_interrupt = %d\n",ASUS_HALL_SENSOR_IRQ);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	__pm_stay_awake(&hall_sensor_dev->wake_src);
	return IRQ_HANDLED;
}


//<ASUS-danielchan201500513>>>>>>>>>+
static void debounce_hall_sensor_report_function(struct work_struct *dat)
{
        unsigned long flags;
		int counter, counter_trigger = 0, initial_status;
		int de_bounce=0;
        int sleep_time=0;
//<ASUS-danielchan201500519>>>>>>>>>+
        if(DEBUG_LOG) printk("[%s] hardcode:%d \n", DRIVER_NAME,hall_sensor_dev->hardcode);
        if(hall_sensor_dev->hardcode==0) {
            input_report_switch(hall_sensor_dev->hall_indev, SW_LID, 0);
            input_sync(hall_sensor_dev->hall_indev);
            return;
        } else if(hall_sensor_dev->hardcode==1) {
            input_report_switch(hall_sensor_dev->hall_indev, SW_LID, 1);
            input_sync(hall_sensor_dev->hall_indev);
            return;
        }
//<ASUS-danielchan20150519<<<<<<<<<<+
        if(DEBUG_LOG) printk("[%s] debounce:%d \n", DRIVER_NAME,hall_sensor_dev->debounce);
        if(DEBUG_LOG) printk("[%s] sleep_time:%d \n", DRIVER_NAME,hall_sensor_dev->sleep);
        if(hall_sensor_dev->debounce<50) {
            log("[%s] debounce<50  \n", DRIVER_NAME);
            de_bounce=50;
        } else {
            de_bounce=hall_sensor_dev->debounce;
        }

        if(hall_sensor_dev->sleep<10  || hall_sensor_dev->sleep>hall_sensor_dev->debounce){
            log("[%s] sleep_time<10 or sleep>debounce \n", DRIVER_NAME);
            sleep_time=10;
        } else {
            sleep_time=hall_sensor_dev->sleep;
        }

        if(!hall_sensor_dev->enable){
            log("[ISR] [%s] hall sensor is disable!\n", DRIVER_NAME);
            __pm_relax(&hall_sensor_dev->wake_src);
            return;
        }
		initial_status =hall_sensor_dev->status;
		if(DEBUG_LOG) printk("[%s] initial_status:%d \n", DRIVER_NAME,hall_sensor_dev->status);
		if(DEBUG_LOG) printk("[%s] de_bounce:%d \n", DRIVER_NAME,de_bounce);
        for (counter = 0;counter < ((de_bounce/sleep_time));counter++) {
			msleep(sleep_time);
	        if(DEBUG_LOG) printk("[hall sensor] counter:%d \n",counter);
            spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
            if (gpio_get_value(ASUS_HALL_SENSOR_GPIO) == 0) { //trigger low 
		        hall_sensor_dev->status = 0;
		        counter_trigger++;
				if(DEBUG_LOG) printk("[%s] gpio_get_value 0 \n", DRIVER_NAME);
            }else{
                hall_sensor_dev->status = 1;
				if(DEBUG_LOG) printk("[%s] gpio_get_value 1 \n", DRIVER_NAME);
		    }
		    spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
        }	
        if(DEBUG_LOG) printk("[%s] counter_trigger:%d \n", DRIVER_NAME,counter_trigger);
        if( (counter_trigger > 0) && (counter_trigger < (de_bounce/sleep_time))){
            log("[%s] SW_LID do not report to framework.\n", DRIVER_NAME);
            hall_sensor_dev->status = initial_status;
            __pm_relax(&hall_sensor_dev->wake_src);
            return;
	    }
        input_report_switch(hall_sensor_dev->hall_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->hall_indev);
        __pm_relax(&hall_sensor_dev->wake_src);
        printk("[hall_sensors][ISR] report value = %d\n", !hall_sensor_dev->status);

}
//<ASUS-danielchan20150513><<<<<<<<+

//wxtest
int report_hall_status(void)
{
	if(!hall_sensor_dev) {
		err("Hall sensor does not exist!\n");
		return -1; //hall sensor is not
	}
	return hall_sensor_dev->status;
}
EXPORT_SYMBOL(report_hall_status);
//wxtest

/*===========================
 *|| sysfs DEVICE_ATTR part ||
 *===========================
 *
 */
static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;
	
        sscanf(buf, "%du", &request);
		
        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        	if (!request)
                	hall_sensor_dev->status = 0;
	 	else
        		hall_sensor_dev->status = 1;
	 spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	
        log("[ATTR] status rewite value = %d\n",!hall_sensor_dev->status);
	return count;
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        if(!hall_sensor_dev)
                return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	
	if(!hall_sensor_dev) {
		err("Hall sensor does not exist!\n");
		return 0;
	}
	
	sscanf(buf, "%du", &request);
	
	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		unsigned long flags;
		if(0 == request) {
			/* Turn off */
			log("[ATTR] Turn off.\n");
			
			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			hall_sensor_dev->enable=request;
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
			
		}else if(1 == request){
			/* Turn on */
			log("[ATTR] Turn on. \n");
			
			spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
			hall_sensor_dev->enable=request;
			spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
			
		}else{
			err("[ATTR] Enable/Disable Error, can not recognize (%d)", request);
		}
		
	}
	return count;
}

static DEVICE_ATTR(status, 0664, show_action_status, store_action_status);
static DEVICE_ATTR(switch, 0664,show_hall_sensor_enable, store_hall_sensor_enable);

static struct attribute *hall_sensor_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_switch.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};


/*====================
 *|| Initialization Part ||
 *====================
 */

static int init_input_event(void)
{
	int ret = 0;

	hall_sensor_dev->hall_indev = input_allocate_device();     
	if(!hall_sensor_dev->hall_indev){
		err("[Input] Failed to allocate input event device\n");
		return -ENOMEM;		
	}

	hall_sensor_dev->hall_indev->name = "hall_input";
	hall_sensor_dev->hall_indev->phys= "/dev/input/hall_indev";
	hall_sensor_dev->hall_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->hall_indev, EV_SW, SW_LID);

	ret = input_register_device(hall_sensor_dev->hall_indev);
	if (ret) {
		err("[Input] Failed to register input event device\n");
		return -1;		
	}
		
	log("[Input] Input Event registration Success!\n");
	return 0;
}

static int init_data(void)
{
	int ret = 0;
	
	/* Memory allocation for data structure */
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		err("Memory allocation fails for hall sensor\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
	wakeup_source_init(&hall_sensor_dev->wake_src, "HallSensor_wake_lock");
	
	hall_sensor_dev->enable = 1;

	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}

static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "hall_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	log("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
}

static int init_irq (void)
{
	int ret = 0;

	/* GPIO to IRQ */
	ASUS_HALL_SENSOR_IRQ = gpio_to_irq(ASUS_HALL_SENSOR_GPIO);
	
	if (ASUS_HALL_SENSOR_IRQ < 0) {
		err("[IRQ] gpio_to_irq ERROR, irq=%d.\n", ASUS_HALL_SENSOR_IRQ);
	}else {
		log("[IRQ] gpio_to_irq IRQ %d successed on GPIO:%d\n", ASUS_HALL_SENSOR_IRQ, ASUS_HALL_SENSOR_GPIO);
	}

	ret = request_threaded_irq(ASUS_HALL_SENSOR_IRQ, NULL, hall_sensor_interrupt_handler,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				INT_NAME, hall_sensor_dev);

	if (ret < 0)
		err("[IRQ] request_irq() ERROR %d.\n", ret);
	else {
		dbg("[IRQ] Enable irq !! \n");
		enable_irq_wake(ASUS_HALL_SENSOR_IRQ);
	}
	
	return 0;
}

//<ASUS-danielchan201500528>>>>>>>>>+
static ssize_t hall_send_cmd_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
    int ret;
    ret = sprintf(buf, "ASUS ---@hall_hardcode:%d \n",hall_sensor_dev->hardcode);
    return ret; 
}

static ssize_t hall_send_cmd_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
    int send_buff = -1;
    sscanf(buf, "%d", &send_buff);
    printk("[%s] hall_send_cmd_write send_buff:%d \n", DRIVER_NAME,send_buff);
    if(send_buff==0) {
        hall_sensor_dev->hardcode=0;
    } else if(send_buff==1) {
        hall_sensor_dev->hardcode=1;
    } else {
        hall_sensor_dev->hardcode=-1;
    }
    return count;
}


static const struct file_operations proc_hall_send = {
     .read       = hall_send_cmd_read,
     .write      = hall_send_cmd_write,
};

static ssize_t debounce_send_cmd_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
    int ret;
    ret = sprintf(buf, "ASUS ---@debounce:%d \n",hall_sensor_dev->debounce);
    return ret;
}

static ssize_t debounce_send_cmd_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
    int send_buff = -1;
    sscanf(buf, "%d", &send_buff);
    printk("[%s] debounce_send_cmd_write send_buff:%d \n", DRIVER_NAME,send_buff);
    hall_sensor_dev->debounce=send_buff;
    return count;
}

static const struct file_operations proc_debounce_send = {
     .read       = debounce_send_cmd_read,
     .write      = debounce_send_cmd_write,
};


static ssize_t sleep_send_cmd_read(struct file *dev, char *buf, size_t count, loff_t *ppos) {
    int ret;
    ret = sprintf(buf, "ASUS ---@sleep:%d \n",hall_sensor_dev->debounce);
    return ret; 
}

static ssize_t sleep_send_cmd_write(struct file *dev,const char *buf, size_t count, loff_t *ppos){
    int send_buff = -1;
    sscanf(buf, "%d", &send_buff);
    log("[%s] sleep_send_cmd_write send_buff:%d \n", DRIVER_NAME,send_buff);
    hall_sensor_dev->sleep=send_buff;
    return count;
}

static const struct file_operations proc_sleep_send = {
     .read       = sleep_send_cmd_read,
     .write      = sleep_send_cmd_write,
};

//<ASUS-danielchan20150528<<<<<<<<<<+

static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;
//<ASUS-danielchan201500528>>>>>>>>>+
    void* dummy = NULL;
    struct proc_dir_entry* proc_hall_data = NULL;
    struct proc_dir_entry* proc_debounce_data = NULL;
    struct proc_dir_entry* proc_sleep_data = NULL;
//<ASUS-danielchan20150603>>>>>>>>>+
#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
    proc_hall_data = proc_create_data("hall_sensor_cmd", 0660, NULL, &proc_hall_send, dummy);
    proc_debounce_data = proc_create_data("hall_sensor_debounce", 0660, NULL, &proc_debounce_send, dummy);
    proc_sleep_data = proc_create_data("hall_sensor_sleep", 0660, NULL, &proc_sleep_send, dummy);
#else
    proc_hall_data=NULL;
    proc_debounce_data=NULL;
    proc_sleep_data=NULL;
    dummy = NULL;
#endif
//<ASUS-danielchan20150603<<<<<<<<<<+
//<ASUS-danielchan20150528<<<<<<<<<<+

log("Probe +++\n");

	/* Initialization Data */
	ret = init_data();
	if (ret < 0)
		goto probe_err;
        
	/* GPIO */
	ASUS_HALL_SENSOR_GPIO = of_get_named_gpio(pdev->dev.of_node, "qcom,hall-gpio", 0);
        log("[GPIO] GPIO =%d(%d)\n", ASUS_HALL_SENSOR_GPIO, gpio_get_value(ASUS_HALL_SENSOR_GPIO));
	gpio_free(ASUS_HALL_SENSOR_GPIO);
	/* GPIO Request */

	set_pinctrl(&pdev->dev);

	ret = gpio_request(ASUS_HALL_SENSOR_GPIO, IRQ_Name);
	if (ret) {
		err("[GPIO] Unable to request gpio %s(%d)\n", IRQ_Name, ASUS_HALL_SENSOR_GPIO);
		goto probe_err;
	}

	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_HALL_SENSOR_GPIO);
	if (ret < 0) {
		err("[GPIO] Unable to set the direction of gpio %d\n", ASUS_HALL_SENSOR_GPIO);
		goto probe_err;
	}

	/* sysfs */
	ret = sysfs_create_group(&pdev->dev.kobj, &hall_sensor_group);
	if (ret) {
		err("Hall sensor sysfs_create_group ERROR.\n");
		goto probe_err;
	}	
//<ASUS-danielchan20150528>>>>>>>>>+
	/* Input Device */
	ret = init_input_event();
	if (ret < 0)
		goto probe_err;
    hall_sensor_dev->hardcode=-1;
    hall_sensor_dev->irq_trigger=0;
     /* IRQ setting*/
    hall_sensor_dev->debounce=100;
    hall_sensor_dev->sleep=50;
     /* Work Queue init */
    hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
    INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, debounce_hall_sensor_report_function);
    ret = init_irq();
    if (ret < 0)
         goto probe_err;
//<ASUS-danielchan20150528><<<<<<<<+
    queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);	
    log("Probe ---\n");
	return 0;

probe_err:
	err("Probe ERROR\n");
	return ret;
	
}

static const struct platform_device_id hall_id_table[] = {
        {DRIVER_NAME, 1},
};

static struct of_device_id hallsensor_match_table[] = {
	{ .compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hallsensor_match_table,
	},
	.probe          = hall_sensor_probe,	
	.id_table	= hall_id_table,
};

//----------------for pm_ops callback----------------

static int __init hall_sensor_init(void)
{	
	int err = 0;
	log("Driver INIT +++\n");

	/************************************************* 
	 * SR device out of function ( L18 power source shut down by camera ) 
	 * without register HALL sensor in SR device
	 */

		err = platform_driver_register(&hall_sensor_driver);
		if (err != 0) {
			err("[platform] platform_driver_register fail, Error : %d\n", err);
                        printk("[Hall Sensor] platform_driver_register fail, Error : %d\n",err);
                }
                if(DEBUG_LOG) printk("[Hall Sensor] platform_driver_register success, Error : %d\n",err);
	
	log("Driver INIT ---\n");

	return err;
}

static void __exit hall_sensor_exit(void)
{
	log("Driver EXIT +++\n");

	free_irq(ASUS_HALL_SENSOR_IRQ, hall_sensor_dev);
	gpio_free(ASUS_HALL_SENSOR_GPIO);	
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	wakeup_source_trash(&hall_sensor_dev->wake_src);
	platform_driver_unregister(&hall_sensor_driver);	
	
	
	log("Driver EXIT ---\n");
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_AUTHOR("sr_Huang <sr_Huang@asus.com>");
MODULE_DESCRIPTION("Intel APX9131 Hall Sensor");
MODULE_LICENSE("GPL v2");

