#ifndef __JY61_H
#define __JY61_H

struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
 
extern char YAWCMD[3] ;
extern char ACCCMD[3] ;
extern char SLEEPCMD[3] ;
extern char UARTMODECMD[3] ;
extern char IICMODECMD[3] ;
								
#endif
