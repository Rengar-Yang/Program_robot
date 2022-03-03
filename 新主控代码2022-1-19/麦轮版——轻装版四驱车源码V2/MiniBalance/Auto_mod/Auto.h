#include "sys.h"

#define front PAin(4)  
#define right PAin(5)
#define back PBin(0)
#define left PAin(1)

extern void GetToQRcode();//前往二维码地点
extern void GetToMateralStorage();//前往原料区
extern void GetToRoughProduct();//前往粗加工区
extern void GetToSemifinishedProduct();//前往半成品区
extern void GetToDestination();//前往终点
extern void PostionInit();
extern void PostionDetect();