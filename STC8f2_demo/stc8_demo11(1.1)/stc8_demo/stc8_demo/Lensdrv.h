#ifndef __LENSDRV_H__
#define __LENSDRV_H__

#include "stc8.h"


typedef volatile union{   //volatile -> 阻止编译器优化该变量
	unsigned char byte;
	struct {
		unsigned char b0:1;
		unsigned char b1:1;
		unsigned char b2:1;
		unsigned char b3:1;
		unsigned char b4:1;
		unsigned char b5:1;
		unsigned char b6:1;
		unsigned char b7:1;
	} flag;
} BITF_8;

typedef volatile union{
	unsigned int word;
	struct {
		unsigned int b0:1;
		unsigned int b1:1;
		unsigned int b2:1;
		unsigned int b3:1;
		unsigned int b4:1;
		unsigned int b5:1;
		unsigned int b6:1;
		unsigned int b7:1;
		unsigned int b8:1;
		unsigned int b9:1;
		unsigned int b10:1;
		unsigned int b11:1;
		unsigned int b12:1;
		unsigned int b13:1;
		unsigned int b14:1;
		unsigned int b15:1;
	} flag;
} BITF_16;




void LensDrvFocusZoomInit();
void LensDrvIrisInit();

extern void LensDrvInit();
extern unsigned int LensDrvSPIReadUint( unsigned char addr );
extern void LensDrvSPIWriteUint( unsigned char addr, unsigned int wt_data );
extern void LensdrvDelay_us(unsigned int x);
extern void LensDrvIrisMove(unsigned int IrsTGT);
extern void LensDrvFocusMove( bit dir, unsigned int FcStep);
extern void LensDrvZoomMove( bit dir, unsigned int ZoomStep);

extern void DO_VD_FZ(void);
extern void MS_Rest(void);

extern void LensDrvIrisOpen(void);
extern void LensDrvFocusSpeed(unsigned int FcSpeed);
extern void LensDrvZoomSpeed(unsigned int ZoomSpeed);
extern void LensDrvFocusStop(void);
extern void LensDrvZoomStop(void);

#endif	/*__LENSDRV_H__*/
