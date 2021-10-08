#include "onsemi_interface.h"
#include "onsemi_i2c.h"
#include "utils.h"
//#include <stdlib.h>
//#include <math.h>

#include "FW/FromCode_05_05_01.h"

#undef  pr_fmt
#define pr_fmt(fmt) "OIS-INTF %s(): " fmt, __func__

/* Burst Length for updating to PMEM */
#define BURST_LENGTH_UC 		( 3*20 ) 	// 60 Total:63Byte Burst
//#define BURST_LENGTH_UC 		( 6*20 ) 	// 120 Total:123Byte Burst
/* Burst Length for updating to Flash */
#define BURST_LENGTH_FC 		( 32 )	 	// 32 Total: 35Byte Burst
//#define BURST_LENGTH_FC 		( 64 )	 	// 64 Total: 67Byte Burst
#define ONE_MSEC_COUNT 15
#define 	MESOF_NUM		2048				
#define 	GYROFFSET_H		( 0x1388 << 16 )//ASUS_BSP Byron temp widen thredshold from 0x06D6 to 0x1388
#define		GSENS			( 4096 << 16 )		
#define		GSENS_MARG		(GSENS / 4)			
#define		POSTURETH		(GSENS - GSENS_MARG)	
#define		ZG_MRGN			(1310 << 16)			
#define		XYG_MRGN		(1024 << 16)	

#define		ACT_CHK_FRQ		0x0008B8E5	
#define		ACT_CHK_NUM		3756		
#define		ACT_THR			0x000003E8	
#define		ACT_MARGIN		0.75f	
#define		GEA_NUM			512				
#define		GEA_DIF_HIG		0x0083			
#define		GEA_DIF_LOW		0x0001	

#define CCI_READ_FAILED -110
//**************************
//	Table of download file
//**************************

UINT_32	ZF7_FW_info[][3] =
{
    /* on Module vendor, Actuator Size,   on vesion number */
    {0x05,             0x01,         VERNUM_05_05_01},
};


const DOWNLOAD_TBL_EXT ZF7_DTbl[] = {
	{0x0501, CcUpdataCode128_05_05_01, UpDataCodeSize_05_05_01,  UpDataCodeCheckSum_05_05_01, CcFromCode128_05_05_01, sizeof(CcFromCode128_05_05_01), FromCheckSum_05_05_01, FromCheckSumSize_05_05_01 },
	{0xFFFF, (void*)0,                 0,                        0,                           (void*)0,               0,                              0,                     0}
};

static int onsemi_servo_on(struct cam_ois_ctrl_t * ctrl);
//static int onsemi_servo_off(struct cam_ois_ctrl_t * ctrl);
//static int onsemi_get_servo_state(struct cam_ois_ctrl_t * ctrl, uint32_t *state);
//static int onsemi_restore_servo_state(struct cam_ois_ctrl_t * ctrl, uint32_t state);

//servo has to be on before ois on
static int onsemi_ois_on(struct cam_ois_ctrl_t * ctrl);
static int onsemi_ois_off(struct cam_ois_ctrl_t * ctrl);
//static int onsemi_ssc_go_off(struct cam_ois_ctrl_t *ctrl);
//********************************************************************************
// Function Name 	: IOWrite32A
//********************************************************************************
uint8_t ZF7_IORead32A( struct cam_ois_ctrl_t *ctrl, uint32_t IOadrs, uint32_t *IOdata )
{
	uint8_t rc = 0;
	rc = onsemi_write_dword( ctrl, CMD_IO_ADR_ACCESS, IOadrs ) ;
	if(rc == 0)
		rc = onsemi_read_dword ( ctrl, CMD_IO_DAT_ACCESS, IOdata ) ;
	return rc;
}

//********************************************************************************
// Function Name 	: IOWrite32A
//********************************************************************************
uint8_t ZF7_IOWrite32A( struct cam_ois_ctrl_t *ctrl, uint32_t IOadrs, uint32_t IOdata )
{
	uint8_t rc = 0;
	rc = onsemi_write_dword( ctrl, CMD_IO_ADR_ACCESS, IOadrs ) ;
	if(rc == 0)
		onsemi_write_dword( ctrl, CMD_IO_DAT_ACCESS, IOdata ) ;
	return rc;
}

//****************************************************
//	CUSTOMER NECESSARY CREATING FUNCTION LIST
//****************************************************
static int ZF7_CntWrt( struct cam_ois_ctrl_t * ctrl, uint8_t* data, uint32_t len)
{
	uint32_t addr = 0;
	int rc = 0;

	if(len <= 2)
	{
		pr_err("len is not larger than 2, check it!\n");
	}

	addr = ( data[0] << 8) + data[1]; //read cmd addr
	rc = onsemi_write_seq_bytes(ctrl, addr, &data[2], len-2);

	if(rc < 0)
	{
		pr_err("write seq byte faild rc = %d\n",rc);
	}
	return rc;
}

int ZF7_CntRd( struct cam_ois_ctrl_t *ctrl, UINT_32 addr, uint8_t * data, UINT_32 len)
{
    int rc = 0;
	if(len <= 2)
	{
		pr_err("len is not larger than 2, check it!\n");
	}

	rc = onsemi_read_seq_bytes(ctrl, addr, data, len);
	if(rc < 0)
	{
		pr_err("read seq byte faild rc = %d\n",rc);
	}
	return rc;
}
//********************************************************************************
// Function Name 	: UnlockCodeSet
//********************************************************************************
UINT_8 ZF7_UnlockCodeSet( struct cam_ois_ctrl_t *ctrl)
{
	UINT_32 UlReadVal, UlCnt=0;

	do {
		ZF7_IOWrite32A( ctrl, 0xE07554, 0xAAAAAAAA );
		ZF7_IOWrite32A( ctrl, 0xE07AA8, 0x55555555 );
		ZF7_IORead32A( ctrl, 0xE07014, &UlReadVal );
		if( (UlReadVal & 0x00000080) != 0 )	return ( 0 ) ;	
		delay_ms( 1 );
	} while( UlCnt++ < 10 );
	return ( 1 );
}

//********************************************************************************
// Function Name 	: UnlockCodeClear
//********************************************************************************
UINT_8 ZF7_UnlockCodeClear(struct cam_ois_ctrl_t *ctrl)
{
	UINT_32 UlDataVal, UlCnt=0;

	do {
		ZF7_IOWrite32A( ctrl, 0xE07014, 0x00000010 );	
		ZF7_IORead32A( ctrl, 0xE07014, &UlDataVal );
		if( (UlDataVal & 0x00000080) == 0 )	return ( 0 ) ;	
		delay_ms( 1 );
	} while( UlCnt++ < 10 );
	return ( 3 );
}
//********************************************************************************
// Function Name 	: WritePermission
//********************************************************************************
void ZF7_WritePermission( struct cam_ois_ctrl_t *ctrl )
{
	ZF7_IOWrite32A( ctrl, 0xE074CC, 0x00000001 );	
	ZF7_IOWrite32A( ctrl, 0xE07664, 0x00000010 );	
}

//********************************************************************************
// Function Name 	: AddtionalUnlockCodeSet
//********************************************************************************
void ZF7_AddtionalUnlockCodeSet( struct cam_ois_ctrl_t *ctrl )
{
	ZF7_IOWrite32A( ctrl, 0xE07CCC, 0x0000ACD5 );	
}
//********************************************************************************
// Function Name 	: CoreResetwithoutMC128
//********************************************************************************
UINT_8 ZF7_CoreResetwithoutMC128( struct cam_ois_ctrl_t *ctrl )
{
	UINT_32	UlReadVal ;
	
	ZF7_IOWrite32A( ctrl, 0xE07554, 0xAAAAAAAA);
	ZF7_IOWrite32A( ctrl, 0xE07AA8, 0x55555555);
	
	ZF7_IOWrite32A( ctrl, 0xE074CC, 0x00000001);
	ZF7_IOWrite32A( ctrl, 0xE07664, 0x00000010);
	ZF7_IOWrite32A( ctrl, 0xE07CCC, 0x0000ACD5);
	ZF7_IOWrite32A( ctrl, 0xE0700C, 0x00000000);
	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000000);
	ZF7_IOWrite32A( ctrl, 0xE07010, 0x00000004);

	delay_ms(100);

	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
	ZF7_IOWrite32A( ctrl, 0xE07014, 0x00000010);

	ZF7_IOWrite32A( ctrl, 0xD00060, 0x00000001 ) ;
	delay_ms( 15 ) ;

	ZF7_IORead32A( ctrl, ROMINFO,				(UINT_32 *)&UlReadVal ) ;	
	switch ( (UINT_8)UlReadVal ){
	case 0x08:
	case 0x0D:
		break;
	
	default:	
		return( 0xE0  | (UINT_8)UlReadVal );
	}
	
	return( 0 );
}

//********************************************************************************
// Function Name 	: PmemUpdate128
//********************************************************************************
UINT_8 ZF7_PmemUpdate128( struct cam_ois_ctrl_t *ctrl, DOWNLOAD_TBL_EXT* ptr )
{
	UINT_8	data[BURST_LENGTH_UC +2 ];
	UINT_16	Remainder;
	const UINT_8 *NcDataVal = ptr->UpdataCode;
	UINT_8	ReadData[8];
	long long CheckSumCode = ptr->SizeUpdataCodeCksm;
	UINT_8 *p = (UINT_8 *)&CheckSumCode;
	UINT_32 i, j;
	UINT_32	UlReadVal, UlCnt , UlNum ;
//--------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------
	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000000);
	onsemi_write_dword( ctrl, 0x3000, 0x00080000 );


	data[0] = 0x40;
	data[1] = 0x00;


	Remainder = ( (ptr->SizeUpdataCode*5) / BURST_LENGTH_UC ); 
	for(i=0 ; i< Remainder ; i++)
	{
		UlNum = 2;
		for(j=0 ; j < BURST_LENGTH_UC; j++){
			data[UlNum] =  *NcDataVal++;
			if( ( j % 5) == 4)	pr_debug("\n");
			UlNum++;
		}
		
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_UC+2 );
	}
	Remainder = ( (ptr->SizeUpdataCode*5) % BURST_LENGTH_UC); 
	if (Remainder != 0 )
	{
		UlNum = 2;
		for(j=0 ; j < Remainder; j++){
			data[UlNum++] = *NcDataVal++;
		}
		ZF7_CntWrt( ctrl, data, Remainder+2 );
	}
	
//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------


	data[0] = 0xF0;											
	data[1] = 0x0E;											
	data[2] = (unsigned char)((ptr->SizeUpdataCode >> 8) & 0x000000FF);	
	data[3] = (unsigned char)(ptr->SizeUpdataCode & 0x000000FF);
	data[4] = 0x00;											
	data[5] = 0x00;											

	ZF7_CntWrt( ctrl,data, 6 ) ;


	UlCnt = 0;
	do{
		delay_ms( 1 );
		if( UlCnt++ > 10 ) {
			ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
			return (0x21) ;									
		}
		onsemi_read_dword( ctrl, 0x0088, &UlReadVal );					
	}while ( UlReadVal != 0 );

	ZF7_CntRd( ctrl, 0xF00E, ReadData , 8 );//TODO
	
	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
	for( i=0; i<8; i++) {
		if(ReadData[7-i] != *p++ ) {  						
			return (0x22) ;				
		}
	}

	return( 0 );
}

//********************************************************************************
// Function Name 	: EraseUserMat128
//********************************************************************************
UINT_8 ZF7_EraseUserMat128(struct cam_ois_ctrl_t *ctrl, UINT_8 StartBlock, UINT_8 EndBlock )
{
	UINT_32 i;
	UINT_32	UlReadVal, UlCnt ;

	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000000);
	onsemi_write_dword( ctrl, 0xF007, 0x00000000 );


	for( i=StartBlock ; i<EndBlock ; i++) {
		onsemi_write_dword( ctrl, 0xF00A, ( i << 10 ) );	
		onsemi_write_dword( ctrl, 0xF00C, 0x00000020 );	


		delay_ms( 5 );
		UlCnt = 0;
		do{

			delay_ms( 1 );
			if( UlCnt++ > 10 ){
				ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
				return (0x31) ;			
			}
			onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );
		}while ( UlReadVal != 0 );
	}
	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
	return(0);

}

//********************************************************************************
// Function Name 	: ProgramFlash128_Standard
//********************************************************************************
UINT_8 ZF7_ProgramFlash128_Standard( struct cam_ois_ctrl_t *ctrl, DOWNLOAD_TBL_EXT* ptr )
{
	UINT_32	UlReadVal, UlCnt , UlNum ;
	UINT_8	data[(BURST_LENGTH_FC + 3)];
	UINT_32 i, j;

	const UINT_8 *NcFromVal = ptr->FromCode + 64;
	const UINT_8 *NcFromVal1st = ptr->FromCode;
	UINT_8 UcOddEvn;

	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000000);
	onsemi_write_dword( ctrl, 0xF007, 0x00000000 );
	onsemi_write_dword( ctrl, 0xF00A, 0x00000010 );
	data[0] = 0xF0;					
	data[1] = 0x08;					
	data[2] = 0x00;					
	
	for(i=1 ; i< ( ptr->SizeFromCode / 64 ) ; i++)
	{
		if( ++UcOddEvn >1 )  	UcOddEvn = 0;	
		if (UcOddEvn == 0) data[1] = 0x08;
		else 			   data[1] = 0x09;		

#if (BURST_LENGTH_FC == 32)
		data[2] = 0x00;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal++;
		}
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_FC+3 ); 
	  	data[2] = 0x20;		
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal++;
		}
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_FC+3 ); 
#elif (BURST_LENGTH_FC == 64)
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal++;
		}
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_FC+3 );  
#endif

		onsemi_write_dword( ctrl, 0xF00B, 0x00000010 );	
		UlCnt = 0;
		if (UcOddEvn == 0){
			do{								
				onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );	
				if( UlCnt++ > 250 ) {
					ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
					return (0x41) ;			
				}
			}while ( UlReadVal  != 0 );			
		 	onsemi_write_dword( ctrl, 0xF00C, 0x00000004 );
		}else{
			do{								
				onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );	
				if( UlCnt++ > 250 ) {
					ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
					return (0x41) ;			
				}
			}while ( UlReadVal  != 0 );			
			onsemi_write_dword( ctrl, 0xF00C, 0x00000008 );	
		}
	}
	UlCnt = 0;
	do{										
		delay_ms( 1 );	
		onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );	
		if( UlCnt++ > 250 ) {
			ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
			return (0x41) ;				
		}
	}while ( (UlReadVal & 0x0000000C) != 0 );	

	{
		onsemi_write_dword( ctrl, 0xF00A, 0x00000000 );	
		data[1] = 0x08;

#if (BURST_LENGTH_FC == 32)
		data[2] = 0x00;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal1st++;
		}
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_FC+3 );
	  	data[2] = 0x20;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal1st++;
		}
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_FC+3 );
#elif (BURST_LENGTH_FC == 64)
		data[2] = 0x00;
		UlNum = 3;
		for(j=0 ; j < BURST_LENGTH_FC; j++){
			data[UlNum++] = *NcFromVal1st++;
		}
		ZF7_CntWrt( ctrl, data, BURST_LENGTH_FC+3 );
#endif

		onsemi_write_dword( ctrl, 0xF00B, 0x00000010 );
		UlCnt = 0;
		do{	
			onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );
			if( UlCnt++ > 250 ) {
				ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
				return (0x41) ;	
			}
		}while ( UlReadVal != 0 );
	 	onsemi_write_dword( ctrl, 0xF00C, 0x00000004 );
	}
	
	UlCnt = 0;	
	do{	
		delay_ms( 1 );	
		onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );
		if( UlCnt++ > 250 ) {
			ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
			return (0x41) ;	
		}
	}while ( (UlReadVal & 0x0000000C) != 0 );	

	ZF7_IOWrite32A( ctrl, 0xE0701C, 0x00000002);
	return( 0 );
}


//********************************************************************************
// Function Name 	: ZF7_FlashMultiRead
//********************************************************************************
UINT_8	ZF7_FlashMultiRead( struct cam_ois_ctrl_t *ctrl, UINT_8 SelMat, UINT_32 UlAddress, UINT_32 *PulData , UINT_8 UcLength )
{
	UINT_8	i	 ;



	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;

	if( UlAddress > 0x000003FF )											return 9;
	
	ZF7_IOWrite32A( ctrl, 0xE07008 , 0x00000000 | (UINT_32)(UcLength-1) );
	ZF7_IOWrite32A( ctrl, 0xE0700C , ((UINT_32)SelMat << 16) | ( UlAddress & 0x00003FFF ) );
	
	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000000);
	ZF7_IOWrite32A( ctrl, 0xE07010 , 0x00000001 );
	for( i=0 ; i < UcLength ; i++ ){
		ZF7_IORead32A( ctrl, 0xE07000 , &PulData[i] ) ;
	}

	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
	return( 0 ) ;
}

//********************************************************************************
// Function Name 	: ZF7_FlashBlockErase
//********************************************************************************
UINT_8	ZF7_FlashBlockErase( struct cam_ois_ctrl_t *ctrl, UINT_8 SelMat , UINT_32 SetAddress )
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;



	if( SelMat != USER_MAT && SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )	return 10;

	if( SetAddress > 0x000003FF )											return 9;


	ans	= ZF7_UnlockCodeSet(ctrl);
	if( ans != 0 )	return( ans ) ;		

	ZF7_WritePermission(ctrl);		
	if( SelMat != USER_MAT ){
		if( SelMat == INF_MAT2 )	ZF7_IOWrite32A( ctrl, 0xE07CCC, 0x00006A4B );
		else						ZF7_IOWrite32A( ctrl, 0xE07CCC, 0x0000C5AD );
	}
	ZF7_AddtionalUnlockCodeSet(ctrl);	
	
	ZF7_IOWrite32A( ctrl, 0xE0700C , ((UINT_32)SelMat << 16) | ( SetAddress & 0x00003C00 )) ;

	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000000);
	ZF7_IOWrite32A( ctrl, 0xE07010 , 4 ) ;

	delay_ms( 5 ) ;

	UlCnt	= 0 ;

	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		ZF7_IORead32A( ctrl, FLASHROM_FLAINT, &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
	ans	= ZF7_UnlockCodeClear(ctrl);	
	if( ans != 0 )	return( ans ) ;	

	return( ans ) ;
}
//********************************************************************************
// Function Name 	: ZF7_FlashBlockWrite
//********************************************************************************
UINT_8	ZF7_FlashBlockWrite( struct cam_ois_ctrl_t *ctrl, UINT_8 SelMat , UINT_32 SetAddress , UINT_32 *PulData)
{
	UINT_32	UlReadVal, UlCnt;
	UINT_8	ans	= 0 ;
	UINT_8	i	 ;

	if( SelMat != INF_MAT0 && SelMat != INF_MAT1 && SelMat != INF_MAT2  )			return 10;
	// 
	if( SetAddress > 0x000003FF )							return 9;

	ans	= ZF7_UnlockCodeSet(ctrl);
	if( ans != 0 )	return( ans ) ;	

	ZF7_WritePermission(ctrl);	
	if( SelMat != USER_MAT ){
		if( SelMat == INF_MAT2 )	ZF7_IOWrite32A( ctrl, 0xE07CCC, 0x00006A4B );
		else						ZF7_IOWrite32A( ctrl, 0xE07CCC, 0x0000C5AD );
	}
	ZF7_AddtionalUnlockCodeSet(ctrl);
	
	ZF7_IOWrite32A( ctrl, 0xE0700C , ((UINT_32)SelMat << 16) | ( SetAddress & 0x000010 )) ;
	
	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000000);
	ZF7_IOWrite32A( ctrl, 0xE07010 , 2 ) ;


	UlCnt	= 0 ;

	for( i=0 ; i< 16 ; i++ ){
		ZF7_IOWrite32A( ctrl, 0xE07004 , PulData[i]  );
	}
	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		ZF7_IORead32A( ctrl, 0xE07018 , &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;

	ZF7_IOWrite32A( ctrl, 0xE07010 , 8  );	
	
	do {
		if( UlCnt++ > 100 ){	ans = 2;	break;	} ;

		ZF7_IORead32A( ctrl, 0xE07018 , &UlReadVal ) ;
	} while( ( UlReadVal & 0x00000080 ) != 0 ) ;
	
	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
	ans	= ZF7_UnlockCodeClear(ctrl);
	return( ans ) ;							

}

//********************************************************************************
// Function Name 	: ZF7_Mat2ReWrite
//********************************************************************************
UINT_8 ZF7_Mat2ReWrite( struct cam_ois_ctrl_t *ctrl )
{
	UINT_32	UlMAT2[32];
	UINT_32	UlCKSUM=0;
	UINT_8	ans , i ;
	UINT_32	UlCkVal ,UlCkVal_Bk;

	ans = ZF7_FlashMultiRead( ctrl, INF_MAT2, 0, UlMAT2, 32 );
	if(ans)	return( 0xA0 );

	if( UlMAT2[FT_REPRG] == PRDCT_WR || UlMAT2[FT_REPRG] == USER_WR ){
		return( 0x00 );
	}
	
	if( UlMAT2[CHECKCODE1] != CHECK_CODE1 )	return( 0xA1 );
	if( UlMAT2[CHECKCODE2] != CHECK_CODE2 )	return( 0xA2 );
	
	for( i=16 ; i<MAT2_CKSM ; i++){
		UlCKSUM += UlMAT2[i];
	}
	if(UlCKSUM != UlMAT2[MAT2_CKSM])		return( 0xA3 );
	
	UlMAT2[FT_REPRG] = USER_WR;
	
	UlCkVal_Bk = 0;
	for( i=0; i < 32; i++ ){
		UlCkVal_Bk +=  UlMAT2[i];
	}
	
	ans = ZF7_FlashBlockErase( ctrl, INF_MAT2 , 0 );
	if( ans != 0 )	return( 0xA4 ) ;		
	
	ans = ZF7_FlashBlockWrite( ctrl, INF_MAT2 , 0 , UlMAT2 );
	if( ans != 0 )	return( 0xA5 ) ;
	ans = ZF7_FlashBlockWrite( ctrl, INF_MAT2 , (UINT_32)0x10 , &UlMAT2[0x10] );
	if( ans != 0 )	return( 0xA5 ) ;

	ans =ZF7_FlashMultiRead( ctrl, INF_MAT2, 0, UlMAT2, 32 );
	if( ans )	return( 0xA0 );
	
	UlCkVal = 0;
	for( i=0; i < 32; i++ ){
		UlCkVal +=  UlMAT2[i];
	}
	
	if( UlCkVal != UlCkVal_Bk )		return( 0xA6 );	
	
	return( 0x01 );	
}

//********************************************************************************
// Function Name 	: FlashUpdate128
//********************************************************************************
UINT_8 ZF7_FlashUpdate128( struct cam_ois_ctrl_t *ctrl, DOWNLOAD_TBL_EXT* ptr )
{
	UINT_8 ans=0;
	UINT_32	UlReadVal, UlCnt ;
	
 	ans = ZF7_CoreResetwithoutMC128(ctrl);
 	if(ans != 0) return( ans );	

	ans = ZF7_Mat2ReWrite(ctrl);
 	if(ans != 0 && ans != 1) return( ans );	
	
 	ans = ZF7_PmemUpdate128( ctrl, ptr );	
	if(ans != 0) return( ans );

//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------
	if( ZF7_UnlockCodeSet(ctrl) != 0 ) 		return (0x33) ;	
	ZF7_WritePermission(ctrl);								
	ZF7_AddtionalUnlockCodeSet(ctrl);						

 	ans = ZF7_EraseUserMat128(ctrl, 0, 10); 
	if(ans != 0){
		if( ZF7_UnlockCodeClear(ctrl) != 0 ) 	return (0x32) ;	
		else					 		return( ans );
	}

 	ans = ZF7_ProgramFlash128_Standard( ctrl, ptr );
	if(ans != 0){
		if( ZF7_UnlockCodeClear(ctrl) != 0 ) 	return (0x43) ;	
		else					 		return( ans );
	}

	if( ZF7_UnlockCodeClear(ctrl) != 0 ) 	return (0x43) ;		

//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------

	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000000);
	onsemi_write_dword( ctrl, 0xF00A, 0x00000000 );				
	onsemi_write_dword( ctrl, 0xF00D, ptr->SizeFromCodeValid );	

	onsemi_write_dword( ctrl, 0xF00C, 0x00000100 );				
	delay_ms( 6 );
	UlCnt = 0;
	do{												
		onsemi_read_dword( ctrl, 0xF00C, &UlReadVal );			
		if( UlCnt++ > 10 ) {
			ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
			return (0x51) ;			
		}
		delay_ms( 1 );		
	}while ( UlReadVal != 0 );

	onsemi_read_dword( ctrl, 0xF00D, &UlReadVal );			

	ZF7_IOWrite32A( ctrl, 0xE0701C , 0x00000002);
	if( UlReadVal != ptr->SizeFromCodeCksm ) {
		return( 0x52 );
	}

	ZF7_IOWrite32A( ctrl, SYSDSP_REMAP,				0x00001000 ) ;	
	delay_ms( 15 ) ;											
	ZF7_IORead32A( ctrl, ROMINFO,				(UINT_32 *)&UlReadVal ) ;	
	if( UlReadVal != 0x0A)		return( 0x53 );

	return( 0 );
}

//********************************************************************************
// Function Name 	: FlashDownload_128
//********************************************************************************
UINT_8 ZF7_FlashDownload128( struct cam_ois_ctrl_t *ctrl, UINT_8 ModuleVendor, UINT_8 ActVer )
{
	DOWNLOAD_TBL_EXT* ptr ;

	ptr = ( DOWNLOAD_TBL_EXT * )ZF7_DTbl ;
	do {
		if( ptr->Index == ( ((UINT_16)ModuleVendor<<8) + ActVer) ) {
			return ZF7_FlashUpdate128( ctrl, ptr );
		}
		ptr++ ;
	} while (ptr->Index != 0xFFFF ) ;

	return 0xF0 ;
}

void	ZF7_MeasFil( struct cam_ois_ctrl_t *ctrl )
{
	UINT_32	UlZF7_MeasFilaA , UlZF7_MeasFilaB , UlZF7_MeasFilaC ;
	UINT_32	UlZF7_MeasFilbA , UlZF7_MeasFilbB , UlZF7_MeasFilbC ;

	UlZF7_MeasFilaA	=	0x7FFFFFFF ;
	UlZF7_MeasFilaB	=	0x00000000 ;
	UlZF7_MeasFilaC	=	0x00000000 ;
	UlZF7_MeasFilbA	=	0x7FFFFFFF ;
	UlZF7_MeasFilbB	=	0x00000000 ;
	UlZF7_MeasFilbC	=	0x00000000 ;


	onsemi_write_dword ( ctrl, 0x8388	, UlZF7_MeasFilaA ) ;
	onsemi_write_dword ( ctrl, 0x8380	, UlZF7_MeasFilaB ) ;
	onsemi_write_dword ( ctrl, 0x8384	, UlZF7_MeasFilaC ) ;

	onsemi_write_dword ( ctrl, 0x8394	, UlZF7_MeasFilbA ) ;
	onsemi_write_dword ( ctrl, 0x838C	, UlZF7_MeasFilbB ) ;
	onsemi_write_dword ( ctrl, 0x8390	, UlZF7_MeasFilbC ) ;

	onsemi_write_dword ( ctrl, 0x83A0	, UlZF7_MeasFilaA ) ;
	onsemi_write_dword ( ctrl, 0x8398	, UlZF7_MeasFilaB ) ;
	onsemi_write_dword ( ctrl, 0x839C	, UlZF7_MeasFilaC ) ;

	onsemi_write_dword ( ctrl, 0x83AC	, UlZF7_MeasFilbA ) ;
	onsemi_write_dword ( ctrl, 0x83A4	, UlZF7_MeasFilbB ) ;
	onsemi_write_dword ( ctrl, 0x83A8	, UlZF7_MeasFilbC ) ;
}

void	ZF7_MemoryClear( struct cam_ois_ctrl_t *ctrl, UINT_16 UsSourceAddress, UINT_16 UsClearSize )
{
	UINT_16	UsLoopIndex ;

	for ( UsLoopIndex = 0 ; UsLoopIndex < UsClearSize ;  ) {
		onsemi_write_dword( ctrl, UsSourceAddress	, 	0x00000000 ) ;	
		UsSourceAddress += 4;
		UsLoopIndex += 4 ;
	}
}

void	ZF7_SetTransDataAdr( struct cam_ois_ctrl_t *ctrl, UINT_16 UsLowAddress , UINT_32 UlLowAdrBeforeTrans )
{
	UnDwdVal	StTrsVal ;

	if( UlLowAdrBeforeTrans < 0x00009000 ){
		StTrsVal.StDwdVal.UsHigVal = (UINT_16)(( UlLowAdrBeforeTrans & 0x0000F000 ) >> 8 ) ;
		StTrsVal.StDwdVal.UsLowVal = (UINT_16)( UlLowAdrBeforeTrans & 0x00000FFF ) ;
	}else{
		StTrsVal.UlDwdVal = UlLowAdrBeforeTrans ;
	}
	onsemi_write_dword( ctrl, UsLowAddress	,	StTrsVal.UlDwdVal );

}

void	ZF7_SetWaitTime( struct cam_ois_ctrl_t *ctrl, UINT_16 UsWaitTime )
{
	onsemi_write_dword( ctrl, 0x0324	, 0 ) ;
	onsemi_write_dword( ctrl, 0x0328	, (UINT_32)(ONE_MSEC_COUNT * UsWaitTime)) ;
}

void	ZF7_ClrMesFil( struct cam_ois_ctrl_t *ctrl )
{
	onsemi_write_dword ( ctrl, 0x02D0	, 0 ) ;
	onsemi_write_dword ( ctrl, 0x02D4	, 0 ) ;

	onsemi_write_dword ( ctrl, 0x02D8	, 0 ) ;
	onsemi_write_dword ( ctrl, 0x02DC	, 0 ) ;

	onsemi_write_dword ( ctrl, 0x02E0	, 0 ) ;
	onsemi_write_dword ( ctrl, 0x02E4	, 0 ) ;

	onsemi_write_dword ( ctrl, 0x02E8	, 0 ) ;
	onsemi_write_dword ( ctrl, 0x02EC	, 0 ) ;
}

void ZF7_MeasAddressSelection( struct cam_ois_ctrl_t *ctrl, UINT_8 mode , INT_32 * measadr_a , INT_32 * measadr_b )
{
	if( mode == 0 ){
		*measadr_a		=	GYRO_RAM_GX_ADIDAT ;	
		*measadr_b		=	GYRO_RAM_GY_ADIDAT ;	
	}else if( mode == 1 ){
		*measadr_a		=	GYRO_ZRAM_GZ_ADIDAT ;	
		*measadr_b		=	ACCLRAM_Z_AC_ADIDAT ;	
	}else{
		*measadr_a		=	ACCLRAM_X_AC_ADIDAT ;	
		*measadr_b		=	ACCLRAM_Y_AC_ADIDAT ;	
	}
}
void	ZF7_MeasureStart( struct cam_ois_ctrl_t *ctrl, INT_32 SlMeasureParameterNum , INT_32 SlMeasureParameterA , INT_32 SlMeasureParameterB )
{
	ZF7_MemoryClear( ctrl, 0x0278 , sizeof( MeasureFunction_Type ) ) ;
	onsemi_write_dword( ctrl, 0x0280	 , 0x80000000 ) ;	
	onsemi_write_dword( ctrl, 0x02A8	 , 0x80000000 ) ;	
	onsemi_write_dword( ctrl, 0x0284	 , 0x7FFFFFFF ) ;	
	onsemi_write_dword( ctrl, 0x02AC	 , 0x7FFFFFFF ) ;	

	ZF7_SetTransDataAdr( ctrl, 0x02A0	, ( UINT_32 )SlMeasureParameterA ) ;	
	ZF7_SetTransDataAdr( ctrl, 0x02C8	, ( UINT_32 )SlMeasureParameterB ) ;	
	onsemi_write_dword( ctrl, 0x0278	 	, 0 ) ;									
	ZF7_ClrMesFil(ctrl) ;													
	ZF7_SetWaitTime(ctrl, 1) ;
	onsemi_write_dword( ctrl, 0x027C		, SlMeasureParameterNum ) ;		
}
void	ZF7_MeasureWait( struct cam_ois_ctrl_t *ctrl )
{
	UINT_32	SlWaitTimerSt ;
	UINT_16	UsTimeOut = 2000;

	do {
		onsemi_read_dword( ctrl, 0x027C, &SlWaitTimerSt ) ;
		UsTimeOut--;
	} while ( SlWaitTimerSt && UsTimeOut );

}
void	ZF7_SetSinWavGenInt( struct cam_ois_ctrl_t *ctrl )
{

	onsemi_write_dword( ctrl, 0x02FC		,	0x00000000 ) ;	
	onsemi_write_dword( ctrl, 0x0300		,	0x60000000 ) ;	
	onsemi_write_dword( ctrl, 0x0304		,	0x00000000 ) ;	

	onsemi_write_dword( ctrl, 0x0310		,	0x00000000 );	
	onsemi_write_dword( ctrl, 0x0314 	,	0x00000000 );	
	onsemi_write_dword( ctrl, 0x0318 	,	0x00000000 );	

	onsemi_write_dword( ctrl, 0x02F4	,	0x00000000 ) ;								// Sine Wave Stop

}
		
UINT_32	ZF7_MeasGyAcOffset(  struct cam_ois_ctrl_t *ctrl  )
{
	UINT_32	UlRsltSts;
	INT_32			SlMeasureParameterA , SlMeasureParameterB ;
	INT_32			SlMeasureParameterNum ;
	UnllnVal		StMeasValueA , StMeasValueB ;
	INT_32			SlMeasureAveValueA[3] , SlMeasureAveValueB[3] ;
	UINT_8			i ;
	INT_32			SlMeasureAZ = 0;

	
	
	ZF7_MeasFil(ctrl ) ;

	SlMeasureParameterNum	=	MESOF_NUM ;
	
	for( i=0 ; i<3 ; i++ )
	{
		ZF7_MeasAddressSelection( ctrl, i, &SlMeasureParameterA , &SlMeasureParameterB );
	
		ZF7_MeasureStart( ctrl, SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;	
		
		ZF7_MeasureWait(ctrl) ;		
	
		onsemi_read_dword( ctrl, 0x0290 		, &StMeasValueA.StUllnVal.UlLowVal ) ;
		onsemi_read_dword( ctrl, 0x0290 + 4	, &StMeasValueA.StUllnVal.UlHigVal ) ;
		onsemi_read_dword( ctrl, 0x02B8 		, &StMeasValueB.StUllnVal.UlLowVal ) ;
		onsemi_read_dword( ctrl, 0x02B8 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
		SlMeasureAveValueA[i] = (INT_32)( (INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB[i] = (INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
	
	}
	
	UlRsltSts = EXE_END ;
	
	if( (SlMeasureAveValueB[1]) >= POSTURETH ){
		SlMeasureAZ = SlMeasureAveValueB[1] - (INT_32)GSENS;
	}else if( (SlMeasureAveValueB[1]) <= -POSTURETH ){
		SlMeasureAZ = SlMeasureAveValueB[1] + (INT_32)GSENS;
	}else{
		UlRsltSts |= EXE_AZADJ ;
	}
	
	if( abs(SlMeasureAveValueA[0]) > GYROFFSET_H )					UlRsltSts |= EXE_GXADJ ;
	if( abs(SlMeasureAveValueB[0]) > GYROFFSET_H ) 					UlRsltSts |= EXE_GYADJ ;
	if( abs(SlMeasureAveValueA[1]) > GYROFFSET_H ) 					UlRsltSts |= EXE_GZADJ ;
//	if(    (SlMeasureAveValueB[1]) < POSTURETH )					UlRsltSts |= EXE_AZADJ ;
	if( abs(SlMeasureAveValueA[2]) > XYG_MRGN )						UlRsltSts |= EXE_AXADJ ;
	if( abs(SlMeasureAveValueB[2]) > XYG_MRGN )						UlRsltSts |= EXE_AYADJ ;
	if( abs( SlMeasureAZ) > ZG_MRGN )								UlRsltSts |= EXE_AZADJ ;


	if( UlRsltSts == EXE_END ){
		onsemi_write_dword( ctrl, GYRO_RAM_GXOFFZ ,		SlMeasureAveValueA[0] ) ;					
		onsemi_write_dword( ctrl, GYRO_RAM_GYOFFZ ,		SlMeasureAveValueB[0] ) ;					
		onsemi_write_dword( ctrl, GYRO_ZRAM_GZOFFZ ,		SlMeasureAveValueA[1] ) ;					
		onsemi_write_dword( ctrl, ACCLRAM_X_AC_OFFSET ,	SlMeasureAveValueA[2] ) ;					
		onsemi_write_dword( ctrl, ACCLRAM_Y_AC_OFFSET ,	SlMeasureAveValueB[2] ) ;					
		onsemi_write_dword( ctrl, ACCLRAM_Z_AC_OFFSET , 	SlMeasureAZ ) ;	

		onsemi_write_dword( ctrl, 0x01D8 , 		0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x01FC , 		0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x0378 , 		0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x019C , 		0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x01C4 , 		0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03C0 + 8 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03F0 + 8 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x0420 + 8 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03C0 + 12 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03F0 + 12 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x0420 + 12 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03C0 + 16 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03F0 + 16 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x0420 + 16 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03C0 + 20 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x03F0 + 20 ,	0x00000000 ) ;		
		onsemi_write_dword( ctrl, 0x0420 + 20 ,	0x00000000 ) ;		
	}
	return( UlRsltSts );
	
		
}

UINT_8 ZF7_WrGyroOffReCalData( struct cam_ois_ctrl_t *ctrl, UINT_8 UcMode )
{
	UINT_32	UlMAT0[32];
	UINT_8 ans = 0, i;
	UINT_16	UsCkVal,UsCkVal_Bk ;
	UINT_32	UlOffset[6];

pr_info( "ZF7_WrGyroOffReCalData : Mode = %d\n", UcMode);
	ans =ZF7_FlashMultiRead( ctrl, INF_MAT0, 0, UlMAT0, 32 );
	if( ans )	return( 1 );
	
	ans = ZF7_FlashBlockErase( ctrl, INF_MAT0 , 0 );
	if( ans != 0 )	return( 2 ) ;			

#if 0
#if DEBUG == 1
	for(i=0;i<32;i++){
pr_info( "[ %d ] = %08x\n", i, UlMAT0[i] );
	}
#endif //DEBUG == 1
#endif
	if( UcMode ){	// write
		onsemi_read_dword( ctrl, GYRO_RAM_GXOFFZ ,		&UlOffset[0] ) ;					
		onsemi_read_dword( ctrl, GYRO_RAM_GYOFFZ ,		&UlOffset[1] ) ;					
		onsemi_read_dword( ctrl, GYRO_ZRAM_GZOFFZ ,		&UlOffset[2] ) ;					
		onsemi_read_dword( ctrl, ACCLRAM_X_AC_OFFSET ,	&UlOffset[3] ) ;					
		onsemi_read_dword( ctrl, ACCLRAM_Y_AC_OFFSET ,	&UlOffset[4] ) ;					
		onsemi_read_dword( ctrl, ACCLRAM_Z_AC_OFFSET , 	&UlOffset[5] ) ;	

		UlMAT0[CALIBRATION_STATUS] &= ~GYRO_REOFF_FLG;
		UlMAT0[G_RE_OFF_XY]			= (UINT_32)(((UINT_32)(( UlOffset[1]>>0 ) & 0xFFFF0000 ))  | ((UINT_32)(( UlOffset[0]>>16 ) & 0x0000FFFF )));
		UlMAT0[G_RE_OFF_Z_AX]		= (UINT_32)(((UINT_32)(( UlOffset[3]>>0 ) & 0xFFFF0000 ))  | ((UINT_32)(( UlOffset[2]>>16 ) & 0x0000FFFF )));
		UlMAT0[A_RE_OFF_YZ]			= (UINT_32)(((UINT_32)(( UlOffset[5]>>0 ) & 0xFFFF0000 ))  | ((UINT_32)(( UlOffset[4]>>16 ) & 0x0000FFFF )));
	}else{
		UlMAT0[CALIBRATION_STATUS] |= GYRO_REOFF_FLG;
		UlMAT0[G_RE_OFF_XY]			= 0xFFFFFFFF;
		UlMAT0[G_RE_OFF_Z_AX]		= 0xFFFFFFFF;
		UlMAT0[A_RE_OFF_YZ]			= 0xFFFFFFFF;
	}
	UsCkVal = 0;
	for( i=0; i < 31; i++ ){
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>0);
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>8);
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>16);
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>24);
	}
	// Remainder
	UsCkVal +=  (UINT_8)(UlMAT0[i]>>0);
	UsCkVal +=  (UINT_8)(UlMAT0[i]>>8);
	UlMAT0[MAT0_CKSM] = ((UINT_32)UsCkVal<<16) | ( UlMAT0[MAT0_CKSM] & 0x0000FFFF);

	ans = ZF7_FlashBlockWrite( ctrl, INF_MAT0 , (UINT_32)0x00 , UlMAT0 );
	if( ans != 0 )	return( 3 ) ;					
	ans = ZF7_FlashBlockWrite( ctrl, INF_MAT0 , (UINT_32)0x10 , &UlMAT0[0x10] );
	if( ans != 0 )	return( 3 ) ;					

	UsCkVal_Bk = UsCkVal;
	ans =ZF7_FlashMultiRead( ctrl, INF_MAT0, 0, UlMAT0, 32 );
	if( ans )	return( 4 );
	
	UsCkVal = 0;
	for( i=0; i < 31; i++ ){
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>0);
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>8);
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>16);
		UsCkVal +=  (UINT_8)(UlMAT0[i]>>24);
	}

	UsCkVal +=  (UINT_8)(UlMAT0[i]>>0);
	UsCkVal +=  (UINT_8)(UlMAT0[i]>>8);
	
pr_info( "[RVAL]:[BVal]=[%04x]:[%04x]\n",UsCkVal, UsCkVal_Bk );
	if( UsCkVal != UsCkVal_Bk )		return(5);
	
pr_info( "ZF7_WrGyroOffReCalData____COMPLETE\n" );
	return(0);
}
#if 0
UINT_8 ZF7_SetAngleCorrection( struct cam_ois_ctrl_t *ctrl, float DegreeGap, UINT_8 SelectAct, UINT_8 Arrangement )
{
	double OffsetAngleS_slt = 0.0f;
	INT_32 Slagx45x = 0, Slagx45y = 0;
	INT_32 Slagy45y = 0, Slagy45x = 0;
	
	if( ( DegreeGap > 180.0f) || ( DegreeGap < -180.0f ) ) return ( 1 );
	if( Arrangement >= 2 ) return ( 1 );

	OffsetAngleS_slt = (double)( +90.0f + DegreeGap ) * 3.141592653589793238 / 180.0f ;
	Slagx45x = (INT_32)( cos( OffsetAngleS_slt )*2147483647.0);
	Slagx45y = (INT_32)(-sin( OffsetAngleS_slt )*2147483647.0);
	Slagy45y = (INT_32)( cos( OffsetAngleS_slt )*2147483647.0);
	Slagy45x = (INT_32)( sin( OffsetAngleS_slt )*2147483647.0);
	onsemi_write_dword( ctrl, 0x86F8 , 			(UINT_32)Slagx45x );
	onsemi_write_dword( ctrl, 0x86FC , 			(UINT_32)Slagx45y );
	onsemi_write_dword( ctrl, 0x8700 , 			(UINT_32)Slagy45y );
	onsemi_write_dword( ctrl, 0x8704 , 			(UINT_32)Slagy45x );


	return ( 0 );
}
#endif

UINT_8	ZF7_RdStatus( struct cam_ois_ctrl_t *ctrl, UINT_8 UcStBitChk )
{
	UINT_32	UlReadVal ;
	int rc = 0;
	rc = onsemi_read_dword( ctrl, 0xF100 , &UlReadVal );
	if( UcStBitChk ){
		UlReadVal &= READ_STATUS_INI ;
	}
	if( !UlReadVal ){
		return( SUCCESS );
	}else{
		return( FAILURE );
	}
}
void	ZF7_OisEna( struct cam_ois_ctrl_t *ctrl )	// OIS ( SMA , VCM ) = ( OFF, ON )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF012 , 0x00000001 ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}
void	ZF7_OisEna_S( struct cam_ois_ctrl_t *ctrl )	// OIS ( SMA , VCM ) = ( ON, OFF )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF012 , 0x00010000 ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}
void	ZF7_OisEna_SV( struct cam_ois_ctrl_t *ctrl )	// OIS ( SMA , VCM ) = ( ON, ON )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF012 , 0x00010001 ) ;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void	ZF7_OisDis( struct cam_ois_ctrl_t *ctrl )	// OIS ( SMA , VCM ) = ( OFF, OFF )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF012 , 0x00000000 ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void	ZF7_SetPanTiltMode( struct cam_ois_ctrl_t *ctrl, UINT_8 UcPnTmod )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	switch ( UcPnTmod ) {
		case 0 :
			onsemi_write_dword( ctrl, 0xF011 ,	0x00000000 ) ;
			break ;
		case 1 :
			onsemi_write_dword( ctrl, 0xF011 ,	0x00000001 ) ;
			break ;
	}

	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void	ZF7_SscEna( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF01C , 0x00000001 ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void	ZF7_SscDis( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF01C , 0x00000000 ) ;
	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

UINT_8	ZF7_TstActMov( struct cam_ois_ctrl_t *ctrl, UINT_8 UcDirSel )
{
	UINT_8	UcRsltSts = 0;
	INT_32	SlMeasureParameterNum ;
	INT_32	SlMeasureParameterA , SlMeasureParameterB ;
	UnllnVal	StMeasValueA  , StMeasValueB ;
	float		SfLimit , Sfzoom , Sflenz , Sfshift ;
	UINT_32		UlLimit , Ulzoom , Ullenz , Ulshift , UlActChkLvl ;
	UINT_8		i;
	UINT_32		UlReturnVal;

	if( UcDirSel == 0x00 ) {							
		onsemi_read_dword( ctrl, Gyro_Limiter_X 			, ( UINT_32 * )&UlLimit ) ;	
		onsemi_read_dword( ctrl, GyroFilterTableX_gxzoom , ( UINT_32 * )&Ulzoom ) ;	
		onsemi_read_dword( ctrl, GyroFilterTableX_gxlenz , ( UINT_32 * )&Ullenz ) ;	
		onsemi_read_dword( ctrl, GyroFilterShiftX 		, ( UINT_32 * )&Ulshift ) ;	
	}else{
		onsemi_read_dword( ctrl, Gyro_Limiter_Y 			, ( UINT_32 * )&UlLimit ) ;	
		onsemi_read_dword( ctrl, GyroFilterTableY_gyzoom , ( UINT_32 * )&Ulzoom ) ;	
		onsemi_read_dword( ctrl, GyroFilterTableY_gylenz , ( UINT_32 * )&Ullenz ) ;	
		onsemi_read_dword( ctrl, GyroFilterShiftY 		, ( UINT_32 * )&Ulshift ) ;	
	}


	SfLimit = (float)UlLimit / (float)0x7FFFFFFF;
	if( Ulzoom == 0){
		Sfzoom = 0;
	}else{
		Sfzoom = (float)abs(Ulzoom) / (float)0x7FFFFFFF;
	}
	if( Ullenz == 0){
		Sflenz = 0;
	}else{
		Sflenz = (float)Ullenz / (float)0x7FFFFFFF;
	}
	Ulshift = ( Ulshift & 0x0000FF00) >> 8 ;	
	Sfshift = 1;
	for( i = 0 ; i < Ulshift ; i++ ){
		Sfshift *= 2;
	}
	UlActChkLvl = (UINT_32)( (float)0x7FFFFFFF * SfLimit * Sfzoom * Sflenz * Sfshift * ACT_MARGIN );

	SlMeasureParameterNum	=	ACT_CHK_NUM ;

	if( UcDirSel == 0x00 ) {								
		SlMeasureParameterA		=	HALL_RAM_HXOFF1 ;		
		SlMeasureParameterB		=	HallFilterD_HXDAZ1 ;	
	} else if( UcDirSel == 0x01 ) {						
		SlMeasureParameterA		=	HALL_RAM_HYOFF1 ;		
		SlMeasureParameterB		=	HallFilterD_HYDAZ1 ;	
	}
	ZF7_SetSinWavGenInt(ctrl);
	
	onsemi_write_dword( ctrl, 0x02FC		,	ACT_CHK_FRQ ) ;		
	onsemi_write_dword( ctrl, 0x0304		,	UlActChkLvl ) ;		
	onsemi_write_dword( ctrl, 0x02F4		,	0x00000001 ) ;		
	if( UcDirSel == 0x00 ) {
		ZF7_SetTransDataAdr( ctrl, 0x030C	,	(UINT_32)HALL_RAM_HXOFF1 ) ;	
	}else if( UcDirSel == 0x01 ){
		ZF7_SetTransDataAdr( ctrl, 0x030C	,	(UINT_32)HALL_RAM_HYOFF1 ) ;	
	}
	onsemi_write_dword ( ctrl, 0x8388	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x8380	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x8384	, 0x78375A71 ) ;

	onsemi_write_dword ( ctrl, 0x8394	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x838C	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x8390	, 0x78375A71 ) ;

	onsemi_write_dword ( ctrl, 0x83A0	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x8398	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x839C	, 0x78375A71 ) ;

	onsemi_write_dword ( ctrl, 0x83AC	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x83A4	, 0x03E452C7 ) ;
	onsemi_write_dword ( ctrl, 0x83A8	, 0x78375A71 ) ;

	ZF7_MeasureStart( ctrl, SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;		
	
	ZF7_MeasureWait(ctrl) ;		
	
	onsemi_write_dword( ctrl, 0x02F4	,	0x00000000 ) ;	
	
	if( UcDirSel == 0x00 ) {
		ZF7_SetTransDataAdr( ctrl, 0x030C	,	(UINT_32)0x00000000 ) ;
		onsemi_write_dword( ctrl, HALL_RAM_HXOFF1		,	0x00000000 ) ;		
	}else if( UcDirSel == 0x01 ){
		ZF7_SetTransDataAdr( ctrl, 0x030C	,	(UINT_32)0x00000000 ) ;
		onsemi_write_dword( ctrl, HALL_RAM_HYOFF1		,	0x00000000 ) ;		
	}
	onsemi_read_dword( ctrl, 0x0298 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	
	onsemi_read_dword( ctrl, 0x0298 + 4 	, &StMeasValueA.StUllnVal.UlHigVal ) ;
	onsemi_read_dword( ctrl, 0x02C0 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	
	onsemi_read_dword( ctrl, 0x02C0 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;


	UlReturnVal = (INT_32)((INT_64)StMeasValueA.UllnValue * 100 / (INT_64)StMeasValueB.UllnValue  ) ;


	
	UcRsltSts = EXE_END ;
	if( UlReturnVal < ACT_THR ){
		if ( !UcDirSel ) {				
			UcRsltSts = EXE_HXMVER ;
		}else{							
			UcRsltSts = EXE_HYMVER ;
		}
	}

	return( UcRsltSts ) ;

}
UINT_8	ZF7_RunHea( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8 	UcRst ;
	UcRst = EXE_END ;
	UcRst |= ZF7_TstActMov( ctrl, 0x00 ) ;
	UcRst |= ZF7_TstActMov( ctrl, 0x01 ) ;
	
	return( UcRst ) ;
}		
 
UINT_8	ZF7_RunGea( struct cam_ois_ctrl_t *ctrl )
{
	UnllnVal	StMeasValueA , StMeasValueB ;
	INT_32		SlMeasureParameterA , SlMeasureParameterB ;
	UINT_8 		UcRst, UcCnt, UcXLowCnt, UcYLowCnt, UcXHigCnt, UcYHigCnt ;
	UINT_16		UsGxoVal[10], UsGyoVal[10], UsDif;
	INT_32		SlMeasureParameterNum , SlMeasureAveValueA , SlMeasureAveValueB ;

	
	UcRst = EXE_END ;
	UcXLowCnt = UcYLowCnt = UcXHigCnt = UcYHigCnt = 0 ;
	
	onsemi_write_dword ( ctrl, 0x8388	, 0x7FFFFFFF ) ;
	onsemi_write_dword ( ctrl, 0x8380	, 0x00000000 ) ;
	onsemi_write_dword ( ctrl, 0x8384	, 0x00000000 ) ;

	onsemi_write_dword ( ctrl, 0x8394	, 0x7FFFFFFF ) ;
	onsemi_write_dword ( ctrl, 0x838C	, 0x00000000 ) ;
	onsemi_write_dword ( ctrl, 0x8390	, 0x00000000 ) ;

	onsemi_write_dword ( ctrl, 0x83A0	, 0x7FFFFFFF ) ;
	onsemi_write_dword ( ctrl, 0x8398	, 0x00000000 ) ;
	onsemi_write_dword ( ctrl, 0x839C	, 0x00000000 ) ;

	onsemi_write_dword ( ctrl, 0x83AC	, 0x7FFFFFFF ) ;
	onsemi_write_dword ( ctrl, 0x83A4	, 0x00000000 ) ;
	onsemi_write_dword ( ctrl, 0x83A8	, 0x00000000 ) ;
	
	for( UcCnt = 0 ; UcCnt < 10 ; UcCnt++ )
	{
	

		SlMeasureParameterNum	=	GEA_NUM ;					
		SlMeasureParameterA		=	GYRO_RAM_GX_ADIDAT ;		
		SlMeasureParameterB		=	GYRO_RAM_GY_ADIDAT ;		
		
		ZF7_MeasureStart( ctrl, SlMeasureParameterNum , SlMeasureParameterA , SlMeasureParameterB ) ;		
	
		ZF7_MeasureWait(ctrl) ;				
	
		onsemi_read_dword( ctrl, 0x0290 		, &StMeasValueA.StUllnVal.UlLowVal ) ;	
		onsemi_read_dword( ctrl, 0x0290 + 4	, &StMeasValueA.StUllnVal.UlHigVal ) ;
		onsemi_read_dword( ctrl, 0x02B8 		, &StMeasValueB.StUllnVal.UlLowVal ) ;	
		onsemi_read_dword( ctrl, 0x02B8 + 4	, &StMeasValueB.StUllnVal.UlHigVal ) ;
	
		SlMeasureAveValueA = (INT_32)( (INT_64)StMeasValueA.UllnValue / SlMeasureParameterNum ) ;
		SlMeasureAveValueB = (INT_32)( (INT_64)StMeasValueB.UllnValue / SlMeasureParameterNum ) ;
		// 
		UsGxoVal[UcCnt] = (UINT_16)( SlMeasureAveValueA >> 16 );	
		
		// 
		UsGyoVal[UcCnt] = (UINT_16)( SlMeasureAveValueB >> 16 );	
		
		
		
		if( UcCnt > 0 )
		{
			if ( (INT_16)UsGxoVal[0] > (INT_16)UsGxoVal[UcCnt] ) {
				UsDif = (UINT_16)((INT_16)UsGxoVal[0] - (INT_16)UsGxoVal[UcCnt]) ;
			} else {
				UsDif = (UINT_16)((INT_16)UsGxoVal[UcCnt] - (INT_16)UsGxoVal[0]) ;
			}
			
			if( UsDif > GEA_DIF_HIG ) {
				UcXHigCnt ++ ;
			}
			if( UsDif < GEA_DIF_LOW ) {
				UcXLowCnt ++ ;
			}
			
			if ( (INT_16)UsGyoVal[0] > (INT_16)UsGyoVal[UcCnt] ) {
				UsDif = (UINT_16)((INT_16)UsGyoVal[0] - (INT_16)UsGyoVal[UcCnt]) ;
			} else {
				UsDif = (UINT_16)((INT_16)UsGyoVal[UcCnt] - (INT_16)UsGyoVal[0]) ;
			}
			
			if( UsDif > GEA_DIF_HIG ) {
				UcYHigCnt ++ ;
			}
			if( UsDif < GEA_DIF_LOW ) {
				UcYLowCnt ++ ;
			}
		}
	}
	
	if( UcXHigCnt >= 1 ) {
		UcRst = UcRst | EXE_GXABOVE ;
	}
	if( UcXLowCnt > 8 ) {
		UcRst = UcRst | EXE_GXBELOW ;
	}
	
	if( UcYHigCnt >= 1 ) {
		UcRst = UcRst | EXE_GYABOVE ;
	}
	if( UcYLowCnt > 8 ) {
		UcRst = UcRst | EXE_GYBELOW ;
	}
	
	
	return( UcRst ) ;
}


void	ZF7_SrvOn( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF010 , 0x00000003 ) ;

	while( UcStRd && ( UlStCnt++ < CNT200MS)) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void	ZF7_SrvOff( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;
	
	onsemi_write_dword( ctrl, 0xF010 , 0x00000000 ) ;

	while( UcStRd && ( UlStCnt++ < CNT050MS)) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void ZF7_VcmStandby( struct cam_ois_ctrl_t *ctrl )
{
	ZF7_IOWrite32A( ctrl, 0xD00078, 0x00000000 );
	ZF7_IOWrite32A( ctrl, 0xD00074, 0x00000010 );
	ZF7_IOWrite32A( ctrl, 0xD00004, 0x00000005 );
	ZF7_IOWrite32A( ctrl, 0xD00064, 0x00000000 );
}

void ZF7_VcmActive( struct cam_ois_ctrl_t *ctrl )
{
	ZF7_IOWrite32A( ctrl, 0xD00004, 0x00000007 );
	ZF7_IOWrite32A( ctrl, 0xD00074, 0x00000000 );
	ZF7_IOWrite32A( ctrl, 0xD00078, 0x00000F3F );
}

void	ZF7_SetStandbyMode( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	onsemi_write_dword( ctrl, 0xF019 ,	0x00000001 ) ;
	while( UcStRd && (UlStCnt++ < CNT100MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void	ZF7_SetActiveMode( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;

	ZF7_IOWrite32A( ctrl, 0xD01008 ,	0x00000090 ) ;
	onsemi_write_dword( ctrl, 0xF019 ,	0x00000000 ) ;	
	while( UcStRd && (UlStCnt++ < CNT100MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}

void ZF7_Standby128to150( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;
	
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01302408	 ) ;
	UcStRd = 1;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01305500	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01304500	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01304400	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---s
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01301101	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
		//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01301305	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01303E00	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}
void ZF7_Active128to150( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;
	
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01301304	 ) ;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x0130111F	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01304400	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01304500	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01305500	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01302408	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
}
void ZF7_DeviceReset128to150( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlStCnt = 0;
	
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01000C01	 ) ;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			goto end; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	delay_ms( 1 ) ;											
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01303201	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS ) ) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			goto end;  
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01302408	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS )) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			goto end; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	end:
		return;
}

UINT_8 ZF7_VcmRemap( struct cam_ois_ctrl_t *ctrl )
{
	UINT_32	UlReadVal ;

	ZF7_IOWrite32A( ctrl, SYSDSP_REMAP,				0x00001000 ) ;	
	delay_ms( 15 ) ;											
	ZF7_IORead32A( ctrl, ROMINFO,				(UINT_32 *)&UlReadVal ) ;	
	if( UlReadVal != 0x0A)		return( 0x53 );
	return( 0 );
}

UINT_8 ZF7_SetPD128to150( struct cam_ois_ctrl_t *ctrl )
{
	UINT_8	UcStRd = 1;
	UINT_32	UlRdData;
	UINT_8	UcRtnval = 0;
	UINT_32	UlStCnt;
	
	ZF7_SetActiveMode(ctrl);
	
	onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01303201	 ) ;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS ) ) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_write_dword( ctrl, CMD_GYRO_RD_ACCS ,0x01000000	 ) ;
	UcStRd = 1;
	UlStCnt = 0;
	while( UcStRd && (UlStCnt++ < CNT050MS ) ) {
		UcStRd = ZF7_RdStatus(ctrl, 1);
				//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
	}
	onsemi_read_dword( ctrl, CMD_GYRO_RD_ACCS ,&UlRdData	 ) ;
	if( UlRdData == 0x00000011 ){
		onsemi_write_dword( ctrl, CMD_GYRO_WR_ACCS ,0x01302408	 ) ;
		UcStRd = 1;
		UlStCnt = 0;
		while( UcStRd && (UlStCnt++ < CNT050MS ) ) {
			UcStRd = ZF7_RdStatus(ctrl, 1);
						//ASUS_BSP Byron Add for Avoid Hang if cci crash +++
			if(ctrl->cci_status == CCI_READ_FAILED) {
				pr_err("%s, cci read failed break directly\n",__func__);
				break; 
			}
			//ASUS_BSP Byron Add for Avoid Hang if cci crash ---
		}
	}else{
		UcRtnval = 1;
	}
	ZF7_SetStandbyMode(ctrl);
	return( UcRtnval );
}

uint8_t ZF7_WaitProcess(struct cam_ois_ctrl_t *ctrl, uint32_t sleep_us, const char * func)
{
	#define MAX_WAIT_TIME_US 500*1000

	struct timeval start_time,current_time;
	uint8_t status = 1;
	uint32_t count = 0;

	do_gettimeofday(&start_time);

	while(1)
	{
		status = ZF7_RdStatus(ctrl, 1);
		if(ctrl->cci_status == CCI_READ_FAILED) {
			pr_err("%s, cci read failed break directly\n",__func__);
			break; 
		}
		if(status == 0)
		{
			do_gettimeofday(&current_time);
			break;
		}
		else
		{
			count++;
			if(sleep_us)
			{
				delay_us(sleep_us);
			}
			do_gettimeofday(&current_time);
			if(diff_time_us(&current_time,&start_time) > MAX_WAIT_TIME_US)
			{
				break;
			}
		}
	}

	if(status == 0 && count)
		pr_info("%s(), wait process done, count %d, sleep %d us each, cost %lld us\n",
						func,count,sleep_us,diff_time_us(&current_time,&start_time));
	else if(status)
		pr_err("%s(), wait process timeout, count %d, sleep %d us each, cost %lld us\n",
						func,count,sleep_us,diff_time_us(&current_time,&start_time));
	return status;
}

int onsemi_is_servo_on(struct cam_ois_ctrl_t * ctrl)
{
	uint32_t servo_state = 0x0;
	onsemi_read_dword(ctrl, 0xF010, &servo_state);
	if(servo_state & 0xFFFFFF00)
		pr_err("High bits in servo state 0x%x is not 0!\n",servo_state);
	return ((servo_state & 0x000000FF) == 0x3);//ignore higher bits
}

static int onsemi_servo_on(struct cam_ois_ctrl_t * ctrl)
{
	onsemi_write_dword( ctrl, 0xF010 , 0x00000003 ) ;//x,y servo both on
	return ZF7_WaitProcess(ctrl,0,__func__);
}
/*
static int onsemi_servo_off(struct cam_ois_ctrl_t * ctrl)
{
	onsemi_write_dword( ctrl, 0xF010 , 0x00000000 ) ;
	return F40_WaitProcess(ctrl,0,__func__);
}


static int onsemi_servo_go_on(struct cam_ois_ctrl_t * ctrl)
{
	if(!onsemi_is_servo_on(ctrl))
	{
		return onsemi_servo_on(ctrl);
	}
	return 0;
}

static int onsemi_servo_go_off(struct cam_ois_ctrl_t * ctrl)
{
	if(onsemi_is_servo_on(ctrl))
	{
		return onsemi_servo_off(ctrl);
	}
	return 0;
}

static int onsemi_get_servo_state(struct cam_ois_ctrl_t * ctrl, uint32_t *state)
{
	return onsemi_read_dword(ctrl,0xF010,state);
}

static int onsemi_restore_servo_state(struct cam_ois_ctrl_t * ctrl, uint32_t state)
{
	int rc;
	uint32_t current_state = 0;

	rc = onsemi_get_servo_state(ctrl,&current_state);
	if(current_state != state)
	{
		onsemi_write_dword(ctrl,0xF010,state);
		rc = F40_WaitProcess(ctrl,0,__func__);
	}
	else
	{
		rc = 0;
	}
	return rc;
}
*/
int onsemi_is_ois_on(struct cam_ois_ctrl_t * ctrl)
{
	uint32_t ois_state = 0x0;
	onsemi_read_dword(ctrl, 0xF012, &ois_state);
	return (ois_state == 0x1) || (ois_state == 0x00010000) || (ois_state == 0x00010001);
}

static int onsemi_ois_on(struct cam_ois_ctrl_t * ctrl)
{
	if(ctrl->soc_info.index == 0)onsemi_write_dword( ctrl, 0xF012 , 0x00010000 ) ;
	else if(ctrl->soc_info.index == 1)onsemi_write_dword( ctrl, 0xF012 , 0x00000001 ) ;
	else pr_err("no match index(%u)\n",ctrl->soc_info.index);
	
	return ZF7_WaitProcess(ctrl,0,__func__);
}
static int onsemi_ois_off(struct cam_ois_ctrl_t * ctrl)
{
	onsemi_write_dword( ctrl, 0xF012 , 0x00000000 ) ;
	return ZF7_WaitProcess(ctrl,0,__func__);
}

int onsemi_ois_go_on(struct cam_ois_ctrl_t * ctrl)
{
	if(!onsemi_is_servo_on(ctrl))
	{
		pr_err("warning: servo state is off!\n");
		onsemi_servo_on(ctrl);//turn on servo before enable ois
	}

	if(!onsemi_is_ois_on(ctrl))
	{
		return onsemi_ois_on(ctrl);
	}
	return 0;
}

int onsemi_ois_go_off(struct cam_ois_ctrl_t * ctrl)
{
	if(onsemi_is_ois_on(ctrl))
	{
		return onsemi_ois_off(ctrl);
	}
	return 0;
}

int onsemi_get_ois_state(struct cam_ois_ctrl_t * ctrl, uint32_t *state)
{
	return onsemi_read_dword(ctrl,0xF012,state);
}

int onsemi_restore_ois_state(struct cam_ois_ctrl_t * ctrl, uint32_t state)
{
	int rc;
	uint32_t current_state = 0;

	rc = onsemi_get_ois_state(ctrl,&current_state);
	if(current_state != state)
	{
		onsemi_write_dword(ctrl,0xF012,state);
		rc = ZF7_WaitProcess(ctrl,0,__func__);
	}
	else
	{
		rc = 0;
	}
	return rc;
}

int onsemi_ssc_go_on(struct cam_ois_ctrl_t *ctrl)
{
	uint32_t ssc_state = 0;
	onsemi_read_dword(ctrl, 0xF01C, &ssc_state);
	if(ssc_state != 0x1)
	{
		onsemi_write_dword( ctrl, 0xF01C , 0x00000001 ) ;
		return ZF7_WaitProcess(ctrl,0,__func__);
	}
	return 0;
}
/*
static int onsemi_ssc_go_off(struct cam_ois_ctrl_t *ctrl)
{
	uint32_t ssc_state = 0;
	onsemi_read_dword(ctrl, 0xF01C, &ssc_state);
	if(ssc_state != 0x0)
	{
		onsemi_write_dword( ctrl, 0xF01C , 0x00000000 ) ;
		return F40_WaitProcess(ctrl,0,__func__);
	}
	return 0;
}
*/
int onsemi_af_dac_setting(struct cam_ois_ctrl_t *ctrl, uint32_t val)
{
	pr_info(" val = %08x\n", val ) ;
	onsemi_write_dword( ctrl, 0xF01A , val ) ;
	return ZF7_WaitProcess(ctrl,0,__func__);
}
void get_module_name_from_fw_id(uint32_t fw_id, char * module_name)
{
	switch(fw_id >>24)
	{
		case 0x01:
			strncpy(module_name, "SEMCO\0", 6);
			break;
		case 0x03:
			strncpy(module_name, "LITEON\0", 7);
			break;
		case 0x05:
			strncpy(module_name, "PRIMAX\0", 7);
			break;
		case 0x10:
			strncpy(module_name, "CHICONY\0",8);
			break;
		case 0x14:
			strncpy(module_name, "HOLITECH\0",9);
			break;
		default:
			strncpy(module_name, "UNKNOWN\0", 8);
	}
	pr_info("module vendor name is %s\n",module_name);
}


void onsemi_dump_state(struct cam_ois_ctrl_t * ctrl, char * state_buf, uint32_t size)
{
	uint32_t servo,ois,ssc,mode,pantilt,focus_self_weight;
	uint32_t chip_id, fw_version,cal_id,actuator;

	onsemi_read_dword(ctrl,0xF010,&servo);
	onsemi_read_dword(ctrl,0xF012,&ois);
	onsemi_read_dword(ctrl,0xF01C,&ssc);
	onsemi_read_dword(ctrl,0xF013,&mode);
	onsemi_read_dword(ctrl,0xF011,&pantilt);
	onsemi_read_dword(ctrl,0xF016,&focus_self_weight);

	onsemi_write_dword(ctrl, 0xC000,0x00D00100);
	onsemi_read_dword(ctrl, 0xD000,&chip_id);

	onsemi_read_dword(ctrl,0x8000,&fw_version);
	onsemi_read_dword(ctrl,0x8004,&cal_id);
	onsemi_read_dword(ctrl,0x8008,&actuator);

	snprintf(state_buf,size,"===chip id 0x%x, fw 0x%x, vcm 0x%x, cal_id 0x%x===\n\
servo 0x%x\nois 0x%x\nssc 0x%x\nmode 0x%x\npantilt 0x%x\nfocus_control 0x%x\n\
=============================================",
    chip_id,fw_version,(actuator&0xFF00)>>8,cal_id,servo,ois,ssc,mode,pantilt,focus_self_weight);
}

void onsemi_check_sequence_read(struct cam_ois_ctrl_t * ctrl)
{
	uint8_t byte[12];
	uint32_t dword[3];
	uint32_t word[6];
	uint32_t byte_one[12];
	uint32_t one_dword = 0x12345678;
	uint8_t * p_uint8 = (uint8_t *)&one_dword;
	int i;

	pr_info("dword 0x%x, byte array: 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			one_dword, p_uint8[0],p_uint8[1],p_uint8[2],p_uint8[3]);
	onsemi_read_seq_bytes(ctrl,0x8000,byte,12);

	onsemi_read_dword(ctrl,0x8000,&dword[0]);
	onsemi_read_dword(ctrl,0x8004,&dword[1]);
	onsemi_read_dword(ctrl,0x8008,&dword[2]);

	onsemi_read_word(ctrl,0x8000,&word[0]);
	onsemi_read_word(ctrl,0x8002,&word[1]);
	onsemi_read_word(ctrl,0x8004,&word[2]);
	onsemi_read_word(ctrl,0x8006,&word[3]);
	onsemi_read_word(ctrl,0x8008,&word[4]);
	onsemi_read_word(ctrl,0x800A,&word[5]);

	pr_info("dword: 0x%08X 0x%08X 0x%08X",dword[0],dword[1],dword[2]);

	pr_info("word: 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X",word[0],word[1],word[2],word[3],word[4],word[5]);

	pr_info("seq byte: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
					  byte[0],byte[1],byte[2],byte[3],byte[4],byte[5],byte[6],byte[7],byte[8],byte[9],byte[10],byte[11]);

	for(i=0;i<12;i++)
	{
		onsemi_read_byte(ctrl,0x8000+i,&byte_one[i]);
	}

	pr_info("byte read: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
					  byte_one[0],byte_one[1],byte_one[2],byte_one[3],byte_one[4],byte_one[5],byte_one[6],byte_one[7],byte_one[8],byte_one[9],byte_one[10],byte_one[11]);

	onsemi_read_seq_bytes(ctrl,0x8000,byte,4);
	pr_info("4 seq byte for 0x8000: 0x%02X 0x%02X 0x%02X 0x%02X",
					  byte[0],byte[1],byte[2],byte[3]);

}

int32_t onsemi_handle_i2c_dword_write(struct cam_ois_ctrl_t * ctrl, struct cam_sensor_i2c_reg_setting * setting)
{
	int i;
	int32_t rc = 0;
	struct cam_sensor_i2c_reg_array * one_write;
	if(setting->data_type == CAMERA_SENSOR_I2C_TYPE_DWORD)
	{
		for(i=0;i<setting->size;i++)
		{
			one_write = &setting->reg_setting[i];
			pr_info("write dword use onsemi_write_dword: reg 0x%04X, data 0x%08X in reg index %d",
					one_write->reg_addr,one_write->reg_data,i);
			if(i>0)
				ZF7_WaitProcess(ctrl,0,__func__);
			rc = onsemi_write_dword(ctrl,one_write->reg_addr,one_write->reg_data);
			if(0xF015 == one_write->reg_addr)
			{
				ZF7_WaitProcess(ctrl,0,__func__);
				pr_info("write 0xF015 and wait finish");
			}
			if(rc < 0)
			{
				pr_err("write dword use onsemi_write_dword: err %d",rc);
				break;
			}
		}
	}
	else
	{
		rc = -1;
		pr_err("data type %d is not dword!",setting->data_type);
	}
	return rc;

}

int onsemi_switch_mode(struct cam_ois_ctrl_t *ctrl, uint8_t mode)
{
	int rc = 0;
	uint32_t reg_addr = 0;
	uint32_t reg_data = 0;

	switch(mode)
	{
		case 0:
			reg_addr = 0xF012;
			reg_data = 0x00000000; //onsemi centering mode, OIS off
			break;
		case 1:
			reg_addr = 0xF013;
			reg_data = 0x00000000; //onsemi movie mode
			break;
		case 2:
			reg_addr = 0xF013;
			reg_data = 0x00000001; //onsemi still mode
			break;
		default:
			pr_err("Not supported ois mode %d\n",mode);
			rc = -1;
	}
	if(rc == 0)
	{
		pr_info("Make sure servo and ssc on before change mode");
		if(mode != 0)
			rc = onsemi_ois_go_on(ctrl);//ois go on
		rc = onsemi_ssc_go_on(ctrl);//ssc on

		rc = onsemi_write_dword(ctrl, reg_addr, reg_data);
		ZF7_WaitProcess(ctrl,0,__func__);
		if(mode == 0 ) pr_info("Mode changed to center\n");
		else if(mode == 1 ) pr_info("Mode changed to movie\n");
		else if(mode == 2 ) pr_info("Mode changed to still\n");
	}

	return rc;
}

//SSC Gain control
#if 0
// OLD Method for lenshift mapping distance
typedef struct
{
	uint32_t lens_shift;
	uint32_t object_distance_cm;
}lens_shift_to_distance_table_t;

static lens_shift_to_distance_table_t g_lens_shift_to_distance_table[] =
{
	{2,1000},
	{4,500},
	{5,400},
	{6,300},
	{9,200},
	{10,185},
	{11,165},
	{12,150},
	{13,140},
	{14,130},
	{15,120},
	{16,110},
	{18,100},
	{20,90},
	{23,80},
	{26,70},
	{30,60},
	{36,50},
	{41,45},
	{46,40},
	{52,35},
	{61,30},
	{73,25},
	{92,20},
	{124,15},
	{133,14},
	{144,13},
	{156,12},
	{171,11},
	{189,10},
	{211,9},
	{238,8},
	{275,7},
	{324,6},
	{395,5}
};

int32_t onsemi_get_50cm_to_10cm_lens_shift(uint32_t* shift_value)
{
	*shift_value = 131;
	return 0;
}
int32_t onsemi_get_10cm_lens_shift(uint32_t* shift_value)
{
	*shift_value = 162;
	return 0;
}
int32_t onsemi_lens_shift_to_distance(uint32_t shift_value, uint32_t* distance_cm)
{
	int i;
	int size = sizeof(g_lens_shift_to_distance_table)/sizeof(lens_shift_to_distance_table_t);
	lens_shift_to_distance_table_t a,b;

	if(shift_value < 2)
	{
		*distance_cm = 1000;
		return 0;
	}
	else if(shift_value > 337)
	{
		*distance_cm = 5;
		return 0;
	}

	for(i=0;i<size;i++)
	{
		if(g_lens_shift_to_distance_table[i].lens_shift == shift_value)
		{
			*distance_cm = g_lens_shift_to_distance_table[i].object_distance_cm;
			return 0;
		}
	}

	for(i=0;i<size-1;i++)
	{
		if(g_lens_shift_to_distance_table[i].lens_shift < shift_value
		   && g_lens_shift_to_distance_table[i+1].lens_shift > shift_value
		  )
		{
			a = g_lens_shift_to_distance_table[i];
			b = g_lens_shift_to_distance_table[i+1];
			*distance_cm = a.object_distance_cm - (shift_value - a.lens_shift)*(a.object_distance_cm - b.object_distance_cm)/(b.lens_shift - a.lens_shift);
			return 0;
		}
	}
	return -1;
}
#endif

static uint32_t g_ssc_gain_map[] =
{
	0x7FFFFFFF,//20mm
	0x7FFFFFFF,//30mm
	0x7FFFFFFF,//40mm
	0x7FFFFFFF,//50mm
	0x7FFFFFFF,//60mm
	0x72492491,//70mm
	0x63FFFFFF,//80mm
	0x58E38E38,//90mm
	0x4FFFFFFF,//100mm
	0x48BA2E8B,//110mm
	0x42AAAAAA,//120mm
	0x3D89D89D,//130mm
	0x39249248,//140mm
	0x35555554,//150mm
	0x31FFFFFF,//160mm
	0x2F0F0F0E,//170mm
	0x2C71C71C,//180mm
	0x2A1AF286,//190mm
	0x27FFFFFF,//200mm
};

int32_t onsemi_config_ssc_gain(struct cam_ois_ctrl_t * ctrl, uint32_t distance_cm)
{
	if(distance_cm > 20)
		return onsemi_write_dword(ctrl,0x8684,0x0);//infinity ssc gain
	else
	{
		if(distance_cm < 2)
			distance_cm = 2;
		return onsemi_write_dword(ctrl,0x8684,g_ssc_gain_map[distance_cm-2]);
	}
}

//ASUS_BSP Lucien +++: Save one Gyro data after doing OIS calibration
int onsemi_gyro_read_xy(struct cam_ois_ctrl_t * ctrl,uint32_t *x_value, uint32_t *y_value)
{
	int rc;

	rc = onsemi_read_dword(ctrl,0x0240,x_value);
	if(rc == 0)
		rc = onsemi_read_dword(ctrl,0x0244,y_value);
	return rc;
}
//ASUS_BSP Lucien ---: Save one Gyro data after doing OIS calibration

int onsemi_read_pair_sensor_data(struct cam_ois_ctrl_t * ctrl,
								 uint32_t reg_addr_x,uint32_t reg_addr_y,
								 uint32_t *value_x,uint32_t *value_y)
{

	int rc;
	rc = onsemi_read_dword(ctrl,reg_addr_x,value_x);
	if(rc == 0)
		rc = onsemi_read_dword(ctrl,reg_addr_y,value_y);
	return rc;
}
//gyro calibration follow
uint8_t onsemi_gyro_calibration(struct cam_ois_ctrl_t * ctrl)
{
	UINT_32 result;

	result = ZF7_MeasGyAcOffset(ctrl);//just K, output data is in pReCalib
	pr_info("ZF7_MeasGyAcOffset result = 0x%02x", result);

	if(0x02 == result)
	{
	     pr_info("ZF7_MeasGyAcOffset success\n");


	     //Go to write gyro data
         result = ZF7_WrGyroOffReCalData(ctrl, 1);
	     if(!result)
	     {
	        pr_info("ZF7_WrGyroOffReCalData success\n");
			return SUCCESS;
	     }
	     else
	     {
	        pr_err("ZF7_WrGyroOffReCalData failed result = 0x%02x\n", result);
			return FAILURE;
	     }
	}
	else
	{
		if(result == 0x40) pr_err("Gyro X error\n");
		if(result == 0x80) pr_err("Gyro Y error\n");
		if(result == 0x400000) pr_err("Gyro Z error\n");
		
		if(result == 0x80000) pr_err("Accelerometer X error\n");
		if(result == 0x100000) pr_err("Accelerometer Y error\n");
		if(result == 0x200000) pr_err("Accelerometer Z error\n");
		return result;
	}

}

//FW update
static int32_t get_target_fw_version(uint32_t module_vendor, uint32_t actuator_id, uint32_t* target_fw_version)
{
	if(module_vendor == VENDOR_ID_PRIMAX)
	{
		if(actuator_id == 0x1)
			*target_fw_version = PRIMAX_VERNUM_VCM_1;
		/*
		else if(actuator_id == 0x2)
			*target_fw_version = PRIMAX_VERNUM_VCM_2;
		*/
		else
			return -2;//invalid actuator id
	}
	else if(module_vendor == VENDOR_ID_LITEON)
	{
		if(actuator_id == 0x0)
			*target_fw_version = LITEON_VERNUM_VCM_0;
		else if(actuator_id == 0x1)
			*target_fw_version = LITEON_VERNUM_VCM_1;
		else if(actuator_id == 0x2)
			*target_fw_version = LITEON_VERNUM_VCM_2;
		else
			return -2;//invalid actuator id
	}
	else
	{
		return -1;//invalid vendor id
	}
	return 0;
}

uint8_t ZF7_need_update_fw(uint32_t current_fw_version, uint32_t actuator_version, uint8_t force_update)
{
	uint8_t module_vendor = current_fw_version>>24;
	uint8_t need_update = 0;
	uint32_t target_fw_version;

	if(current_fw_version == 0)
	{
		pr_info("FW Bad, need update to save..\n");
		return 1;//Bad FW, need save
	}

	if((module_vendor == VENDOR_ID_PRIMAX && current_fw_version < PRIMAX_VERNUM_BASE) ||
	   (module_vendor == VENDOR_ID_LITEON && current_fw_version < LITEON_VERNUM_BASE)
	  )
	{
		pr_err("fw version 0x%x older than base version, NOT update FW\n",current_fw_version);
		return 0;
	}

	if(get_target_fw_version(module_vendor,(actuator_version & 0xff00) >> 8,&target_fw_version) < 0)
	{
		pr_err("Module invalid! module id 0x%x, actuator id 0x%x\n",module_vendor,(actuator_version & 0xff00) >> 8);
		need_update = 0;
	}
	else
	{
		if(force_update)
		{
			pr_info("Going force update FW, 0x%x -> 0x%x",current_fw_version,target_fw_version);
			need_update = 1;
		}
		else
		{
			if(current_fw_version >= target_fw_version)
			{
				pr_info("fw version 0x%x is latest, NOT update FW",current_fw_version);
				need_update = 0;
			}
			else
			{
				pr_info("Going update FW, 0x%x -> 0x%x",current_fw_version,target_fw_version);
				need_update = 1;
			}
		}
	}
	return need_update;
}

int32_t ZF7_update_fw(struct cam_ois_ctrl_t *ctrl, uint32_t mode,
										uint8_t module_vendor, uint8_t vcm, uint32_t* updated_version)
{
	int32_t rc = 0;
	uint32_t chipid = 0;
	//uint32_t checksum_status;
	uint32_t fw_version_after;
	int32_t update_result;

	pr_info("vendor 0x%x, vcm 0x%x, mode %d\n",module_vendor,vcm,mode);

	update_result = ZF7_FlashDownload128(ctrl, module_vendor, vcm);

	if(update_result != 0)
	{
	   pr_err("ZF7_FlashDownload128 fail! rc 0x%x\n", rc);
	   delay_ms(50);
	}
	else
	{
		//check if really OK
		pr_info("ZF7_FlashDownload128 Succeeded!\n");
		delay_ms(100);
		ZF7_WaitProcess(ctrl,0,__func__);
		rc = ZF7_IORead32A(ctrl,0x00D00100,&chipid);
		if(rc == 0)
		{
			pr_info("chipid id 0x%04X\n",chipid);
			if(chipid != 0x132)
			{
				pr_err("chip id not matched!\n");
				update_result = -1;
			}
		}
		else
		{
			pr_err("read chip id failed! rc = %d\n",rc);
			update_result = -2;
		}
#if 0
		if(rc == 0)
			rc = ZF7_IORead32A(ctrl,0x0000000C,&checksum_status);
		if(rc == 0)
		{
			if(checksum_status != 0)
			{
				pr_info("checksum status 0x%x, not matched! update FW failed!\n",checksum_status);
				update_result = -3;
			}
		}
		else
		{
			pr_err("read checksum status failed! rc = %d\n",rc);
			update_result = -2;
		}
#endif
	}
	rc = onsemi_read_dword(ctrl,0x8000,&fw_version_after);
	if(rc == 0)
	{
		pr_info("After update firmware, version 0x%04X\n",fw_version_after);
		*updated_version = fw_version_after;
	}
	else
	{
		pr_err("read fw version failed!\n");
	}
	return update_result;
}
int32_t onsemi_OV08A_poweroff_setting(struct cam_ois_ctrl_t * ctrl)
{
	int32_t rc = 0;
	ZF7_SscDis(ctrl); //SSC off
	if(ctrl->cci_status == CCI_READ_FAILED) goto end;
	ZF7_OisDis(ctrl);//OIS off
	if(ctrl->cci_status == CCI_READ_FAILED) goto end;
	onsemi_write_dword(ctrl, 0xF01F, 0x00000000);//SMA off
	if(ctrl->cci_status == CCI_READ_FAILED) goto end;
	ZF7_SrvOff(ctrl);//VCM off
	if(ctrl->cci_status == CCI_READ_FAILED) goto end;
	ZF7_DeviceReset128to150(ctrl);
	if(ctrl->cci_status == CCI_READ_FAILED) goto end;
	rc = ZF7_VcmRemap(ctrl);

end:
	if(ctrl->cci_status == CCI_READ_FAILED) {
		pr_err("cci status(%d) error\n",ctrl->cci_status);
	}
	if(rc != 0)
		pr_err("%s: failed",__func__);
	
	return rc;
}
