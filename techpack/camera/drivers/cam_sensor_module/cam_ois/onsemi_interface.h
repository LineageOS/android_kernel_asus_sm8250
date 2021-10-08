#ifndef ASUS_OIS_ONSEMI_INTERFACE_H
#define ASUS_OIS_ONSEMI_INTERFACE_H

#include "cam_ois_dev.h"
#include <linux/types.h>
#define	LITEON_VERNUM_VCM_0  0x03130204
#define	LITEON_VERNUM_VCM_1  0x03130204
#define	LITEON_VERNUM_VCM_2  0x03130206

#define	PRIMAX_VERNUM_VCM_1  0x05130504
#define	PRIMAX_VERNUM_VCM_2  0x05130206

#define LITEON_VERNUM_BASE 	0x03130201
#define PRIMAX_VERNUM_BASE 	0x05130501

#define VENDOR_ID_LITEON    0x03
#define VENDOR_ID_PRIMAX    0x05
#define VENDOR_ID_HOLITECH    0x14


typedef	int8_t		INT_8;
typedef	int16_t		INT_16;
typedef	int32_t     INT_32;
typedef	int64_t     INT_64;
typedef	uint8_t     UINT_8;
typedef	uint16_t    UINT_16;
typedef	uint32_t    UINT_32;
typedef	uint64_t	UINT_64;

//****************************************************
//	STRUCTURE DEFINE
//****************************************************
typedef struct {
	UINT_16				Index;
	const UINT_8*		UpdataCode;
	UINT_32				SizeUpdataCode;
	UINT_64				SizeUpdataCodeCksm;
	const UINT_8*		FromCode;
	UINT_32				SizeFromCode;
	UINT_64				SizeFromCodeCksm;
	UINT_32				SizeFromCodeValid;
}	DOWNLOAD_TBL_EXT;

typedef struct STRECALIB {
	INT_16	SsFctryOffX ;
	INT_16	SsFctryOffY ;
	INT_16	SsRecalOffX ;
	INT_16	SsRecalOffY ;
	INT_16	SsDiffX ;
	INT_16	SsDiffY ;
} stReCalib ;

typedef struct {
	INT_32				SiSampleNum ;	
	INT_32				SiSampleMax ;	

	struct {
		INT_32			SiMax1 ;		
		INT_32			SiMin1 ;		
		UINT_32	UiAmp1 ;				
		INT_64		LLiIntegral1 ;		
		INT_64		LLiAbsInteg1 ;		
		INT_32			PiMeasureRam1 ;	
	} MeasureFilterA ;

	struct {
		INT_32			SiMax2 ;		
		INT_32			SiMin2 ;		
		UINT_32	UiAmp2 ;				
		INT_64		LLiIntegral2 ;		
		INT_64		LLiAbsInteg2 ;		
		INT_32			PiMeasureRam2 ;	
	} MeasureFilterB ;
} MeasureFunction_Type ;

union	DWDVAL {
	UINT_32	UlDwdVal ;
	UINT_16	UsDwdVal[ 2 ] ;
	struct {
		UINT_16	UsLowVal ;
		UINT_16	UsHigVal ;
	} StDwdVal ;
	struct {
		UINT_8	UcRamVa0 ;
		UINT_8	UcRamVa1 ;
		UINT_8	UcRamVa2 ;
		UINT_8	UcRamVa3 ;
	} StCdwVal ;
} ;

typedef union DWDVAL	UnDwdVal;

union	ULLNVAL {
	UINT_64	UllnValue ;
	UINT_32	UlnValue[ 2 ] ;
	struct {
		UINT_32	UlLowVal ;
		UINT_32	UlHigVal ;
	} StUllnVal ;
} ;

typedef union ULLNVAL	UnllnVal;

#define		EXE_END		0x00000002L	
#define		EXE_GXADJ	0x00000042L	
#define		EXE_GYADJ	0x00000082L	
#define		EXE_GZADJ	0x00400002L	
#define		EXE_AZADJ	0x00200002L	
#define		EXE_AYADJ	0x00100002L	
#define		EXE_AXADJ	0x00080002L	
#define		EXE_HXMVER	0x06
#define		EXE_HYMVER	0x0A
#define		EXE_GXABOVE	0x06
#define		EXE_GXBELOW	0x0A
#define		EXE_GYABOVE	0x12
#define		EXE_GYBELOW	0x22

#define		GYRO_REOFF_FLG					0x00002000

#define	SUCCESS			0x00
#define	FAILURE			0x01

#define	CALIBRATION_STATUS		(  0 )
#define	G_RE_OFF_XY				( 23 )
#define	G_RE_OFF_Z_AX			( 24 )
#define	A_RE_OFF_YZ				( 25 )
#define	MAT0_CKSM				( 31 )

#define	FT_REPRG				( 15 )
	#define	PRDCT_WR				0x55555555
	#define	USER_WR					0xAAAAAAAA
#define	MAT2_CKSM				( 29 )
#define	CHECKCODE1				( 30 )
	#define	CHECK_CODE1				0x99756768
#define	CHECKCODE2				( 31 )
	#define	CHECK_CODE2				0x01AC28AC
	
//==============================================================================
//
//==============================================================================
#define		CMD_IO_ADR_ACCESS				0xC000				//!< IO Write Access
#define		CMD_IO_DAT_ACCESS				0xD000				//!< IO Read Access
#define 	SYSDSP_DSPDIV					0xD00014
#define 	SYSDSP_SOFTRES					0xD0006C
#define 	SYSDSP_REMAP					0xD000AC
#define 	SYSDSP_CVER						0xD00100
#define		ROMINFO							0xE050D4
#define FLASHROM_128		0xE07000	// Flash Memory I/F°t¸m????
#define 		FLASHROM_FLA_RDAT					(FLASHROM_128 + 0x00)
#define 		FLASHROM_FLA_WDAT					(FLASHROM_128 + 0x04)
#define 		FLASHROM_ACSCNT						(FLASHROM_128 + 0x08)
#define 		FLASHROM_FLA_ADR					(FLASHROM_128 + 0x0C)
	#define			USER_MAT				0
	#define			INF_MAT0				1
	#define			INF_MAT1				2
	#define			INF_MAT2				4
#define 		FLASHROM_CMD						(FLASHROM_128 + 0x10)
#define 		FLASHROM_FLAWP						(FLASHROM_128 + 0x14)
#define 		FLASHROM_FLAINT						(FLASHROM_128 + 0x18)
#define 		FLASHROM_FLAMODE					(FLASHROM_128 + 0x1C)
#define 		FLASHROM_TPECPW						(FLASHROM_128 + 0x20)
#define 		FLASHROM_TACC						(FLASHROM_128 + 0x24)

#define 		FLASHROM_ERR_FLA					(FLASHROM_128 + 0x98)
#define 		FLASHROM_RSTB_FLA					(FLASHROM_128 + 0x4CC)
#define 		FLASHROM_UNLK_CODE1					(FLASHROM_128 + 0x554)
#define 		FLASHROM_CLK_FLAON					(FLASHROM_128 + 0x664)
#define 		FLASHROM_UNLK_CODE2					(FLASHROM_128 + 0xAA8)
#define 		FLASHROM_UNLK_CODE3					(FLASHROM_128 + 0xCCC)

#define		READ_STATUS_INI					0x01000000

#define			HallFilterD_HXDAZ1			0x0048
#define			HallFilterD_HYDAZ1			0x0098

#define			HALL_RAM_HXOFF              0x00D8
#define			HALL_RAM_HYOFF				0x0128
#define			HALL_RAM_HXOFF1				0x00DC
#define			HALL_RAM_HYOFF1				0x012C
#define			HALL_RAM_HXOUT0				0x00E0
#define			HALL_RAM_HYOUT0				0x0130
#define			HALL_RAM_SINDX1				0x00F0
#define			HALL_RAM_SINDY1				0x0140
#define			HALL_RAM_HALL_X_OUT			0x00F4
#define			HALL_RAM_HALL_Y_OUT			0x0144
#define			HALL_RAM_HXIDAT				0x0178
#define			HALL_RAM_HYIDAT				0x017C
#define			HALL_RAM_GYROX_OUT			0x0180
#define			HALL_RAM_GYROY_OUT			0x0184
#define			HallFilterCoeffX_hxgain0	0x80F0
#define			HallFilterCoeffY_hygain0	0x818C
#define			Gyro_Limiter_X				0x8330
#define			Gyro_Limiter_Y   	        0x8334
#define			GyroFilterTableX_gxzoom		0x82B8
#define			GyroFilterTableY_gyzoom		0x8318
#define			GyroFilterTableX_gxlenz		0x82BC
#define			GyroFilterTableY_gylenz		0x831C
#define			GyroFilterShiftX			0x8338
#define			GyroFilterShiftY			0x833C

#define			GYRO_RAM_GX_ADIDAT			0x0220
#define			GYRO_RAM_GY_ADIDAT			0x0224
#define			GYRO_RAM_GXOFFZ				0x0240
#define			GYRO_RAM_GYOFFZ				0x0244
#define			GYRO_ZRAM_GZ_ADIDAT			0x039C
#define			GYRO_ZRAM_GZOFFZ			0x03A8
#define			ACCLRAM_X_AC_ADIDAT			0x0450
#define			ACCLRAM_X_AC_OFFSET			0x0454
#define			ACCLRAM_Y_AC_ADIDAT			0x047C
#define			ACCLRAM_Y_AC_OFFSET			0x0480
#define			ACCLRAM_Z_AC_ADIDAT			0x04A8
#define			ACCLRAM_Z_AC_OFFSET			0x04AC


/************************************************/
/*	Command										*/
/************************************************/
#define		CMD_IO_ADR_ACCESS				0xC000			
#define		CMD_IO_DAT_ACCESS				0xD000			
#define		CMD_RETURN_TO_CENTER			0xF010			
	#define		BOTH_SRV_OFF					0x00000000	
	#define		XAXS_SRV_ON						0x00000001	
	#define		YAXS_SRV_ON						0x00000002	
	#define		BOTH_SRV_ON						0x00000003	
#define		CMD_PAN_TILT					0xF011			
	#define		PAN_TILT_OFF					0x00000000	
	#define		PAN_TILT_ON						0x00000001	
#define		CMD_OIS_ENABLE					0xF012			
	#define		OIS_DISABLE						0x00000000	
	#define		OIS_ENABLE						0x00000001	
	#define		SMA_OIS_ENABLE					0x00010000	
	#define		BOTH_OIS_ENABLE					0x00010001	
	#define		OIS_ENABLE_LF					0x00000011	
	#define		SMA_OIS_ENABLE_LF				0x00010010	
	#define		BOTH_OIS_ENABLE_LF				0x00010011	
#define		CMD_MOVE_STILL_MODE				0xF013			
	#define		MOVIE_MODE						0x00000000	
	#define		STILL_MODE						0x00000001	
	#define		MOVIE_MODE1						0x00000002	
	#define		STILL_MODE1						0x00000003	
	#define		MOVIE_MODE2						0x00000004	
	#define		STILL_MODE2						0x00000005	
	#define		MOVIE_MODE3						0x00000006	
	#define		STILL_MODE3						0x00000007	
#define		CMD_GYROINITIALCOMMAND			0xF015			
	#define		SET_ICM20690					0x00000000
	#define		SET_LSM6DSM						0x00000002
	#define		SET_BMI260						0x00000006
#define		CMD_OSC_DETECTION				0xF017			
	#define		OSC_DTCT_DISABLE				0x00000000
	#define		OSC_DTCT_ENABLE					0x00000001
#define		CMD_STANDBY_ENABLE				0xF019
	#define		ACTIVE_MODE						0x00000000
	#define		STANDBY_MODE					0x00000001
#define		CMD_SSC_ENABLE					0xF01C			
	#define		SSC_DISABLE						0x00000000	
	#define		SSC_ENABLE						0x00000001	
#define		CMD_GYRO_RD_ACCS				0xF01D			
#define		CMD_GYRO_WR_ACCS				0xF01E			
#define		CMD_SMA_CONTROL					0xF01F
	#define		SMA_STOP						0x00000000
	#define		SMA_START						0x00000001
#define		CMD_READ_STATUS					0xF100			
#define			READ_STATUS_INI					0x01000000

#define		CNT050MS		 5//676
#define		CNT100MS		1352
#define		CNT200MS		2703

//==============================================================================
// Prototype
//==============================================================================
UINT_8	ZF7_FlashDownload128( struct cam_ois_ctrl_t *ctrl, UINT_8 , UINT_8  );

UINT_8	ZF7_SetAngleCorrection( struct cam_ois_ctrl_t *ctrl, float , UINT_8 , UINT_8  );
UINT_8	ZF7_UnlockCodeSet( struct cam_ois_ctrl_t *ctrl );
UINT_8	ZF7_UnlockCodeClear(struct cam_ois_ctrl_t *ctrl);
UINT_32	ZF7_MeasGyAcOffset(  struct cam_ois_ctrl_t *ctrl  );

UINT_8	ZF7_RdStatus( struct cam_ois_ctrl_t *ctrl, UINT_8 UcStBitChk );
void	ZF7_OisEna( struct cam_ois_ctrl_t *ctrl );
void	ZF7_OisDis( struct cam_ois_ctrl_t *ctrl );
void	ZF7_OisEna_S( struct cam_ois_ctrl_t *ctrl );
void	ZF7_OisEna_SV( struct cam_ois_ctrl_t *ctrl );
void	ZF7_SetPanTiltMode( struct cam_ois_ctrl_t *ctrl, UINT_8 UcPnTmod );
void	ZF7_SscEna( struct cam_ois_ctrl_t *ctrl );
void	ZF7_SscDis( struct cam_ois_ctrl_t *ctrl );

UINT_8	ZF7_RunHea( struct cam_ois_ctrl_t *ctrl );
UINT_8	ZF7_RunGea( struct cam_ois_ctrl_t *ctrl );

//UINT_32	ZF7_FW_info[][3];

void	ZF7_VcmStandby( struct cam_ois_ctrl_t *ctrl );
void	ZF7_VcmActive( struct cam_ois_ctrl_t *ctrl );
void	ZF7_SrvOn( struct cam_ois_ctrl_t *ctrl );
void	ZF7_SrvOff( struct cam_ois_ctrl_t *ctrl );
void	ZF7_SetStandbyMode( struct cam_ois_ctrl_t *ctrl );
void	ZF7_SetActiveMode( struct cam_ois_ctrl_t *ctrl );
void	ZF7_Standby128to150( struct cam_ois_ctrl_t *ctrl );
void	ZF7_Active128to150( struct cam_ois_ctrl_t *ctrl );
void	ZF7_DeviceReset128to150( struct cam_ois_ctrl_t *ctrl );
UINT_8	ZF7_VcmRemap( struct cam_ois_ctrl_t *ctrl );
UINT_8	ZF7_SetPD128to150( struct cam_ois_ctrl_t *ctrl );
UINT_8	ZF7_WrGyroOffReCalData( struct cam_ois_ctrl_t *ctrl , UINT_8);

//OLD Prototype
#if 1
int onsemi_is_ois_on(struct cam_ois_ctrl_t * ctrl);
int onsemi_ois_go_on(struct cam_ois_ctrl_t * ctrl);
int onsemi_ois_go_off(struct cam_ois_ctrl_t * ctrl);
int onsemi_get_ois_state(struct cam_ois_ctrl_t * ctrl, uint32_t *state);
int onsemi_restore_ois_state(struct cam_ois_ctrl_t * ctrl, uint32_t state);
int onsemi_is_servo_on(struct cam_ois_ctrl_t * ctrl);
int onsemi_ssc_go_on(struct cam_ois_ctrl_t *ctrl);

void get_module_name_from_fw_id(uint32_t fw_id, char * module_name);

void onsemi_dump_state(struct cam_ois_ctrl_t * ctrl, char * state_buf, uint32_t size);
void onsemi_check_sequence_read(struct cam_ois_ctrl_t * ctrl);

int onsemi_switch_mode(struct cam_ois_ctrl_t *ctrl, uint8_t mode);

//ASUS_BSP Lucien +++: Save one Gyro data after doing OIS calibration
int onsemi_gyro_read_xy(struct cam_ois_ctrl_t * ctrl, uint32_t *x_value, uint32_t *y_value);
//ASUS_BSP Lucien ---: Save one Gyro data after doing OIS calibration
int onsemi_af_dac_setting(struct cam_ois_ctrl_t *ctrl, uint32_t val);
int onsemi_read_pair_sensor_data(struct cam_ois_ctrl_t * ctrl,
								 uint32_t reg_addr_x,uint32_t reg_addr_y,
								 uint32_t *value_x,uint32_t *value_y);

uint8_t onsemi_gyro_calibration(struct cam_ois_ctrl_t * ctrl);

uint8_t ZF7_need_update_fw(uint32_t current_fw_version, uint32_t actuator_version, uint8_t force_update);
int32_t ZF7_update_fw(struct cam_ois_ctrl_t *ctrl, uint32_t mode, uint8_t module_vendor, uint8_t vcm, uint32_t* updated_version);

uint8_t ZF7_IORead32A(struct cam_ois_ctrl_t *ctrl, uint32_t IOadrs, uint32_t*IOdata);
uint8_t ZF7_WaitProcess(struct cam_ois_ctrl_t *ctrl, uint32_t sleep_us, const char * func);

int32_t onsemi_handle_i2c_dword_write(struct cam_ois_ctrl_t * ctrl,struct cam_sensor_i2c_reg_setting * setting);
int32_t onsemi_get_50cm_to_10cm_lens_shift(uint32_t* shift_value);
int32_t onsemi_get_10cm_lens_shift(uint32_t* shift_value);
int32_t onsemi_lens_shift_to_distance(uint32_t shift_value, uint32_t* distance_cm);
int32_t onsemi_config_ssc_gain(struct cam_ois_ctrl_t * ctrl, uint32_t distance_cm);
int32_t onsemi_OV08A_poweroff_setting(struct cam_ois_ctrl_t * ctrl);
#endif
#endif
