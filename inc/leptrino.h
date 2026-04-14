#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>

#include "pCommon.h"
#include "rs_comm.h"
#include "pComResInternal.h"

#define PRG_VER	"Ver 1.0.0"

#define MAX_BUFF		10
#define MAX_LENGTH	255

#define STS_IDLE		0
#define STS_WAIT_STX	1
#define STS_DATA		2
#define STS_WAIT_ETX	3
#define STS_WAIT_BCC	4

class leptrino
{
public:
    int i, l = 0, rt = 0;
	int mode_step = 0;
	int AdFlg = 0, EndF = 0;
	long cnt = 0;
	UCHAR strprm[256];
	ST_RES_HEAD *stCmdHead;
	ST_R_DATA_GET_F *stForce;
	ST_R_GET_INF *stGetInfo;

	int Comm_RcvF=0;								//��M�f�[�^�L�t���O
	int p_rd=0,p_wr=0;							//��M�����O�o�b�t�@�Ǐo���A�����݃|�C���^
	int fd=0;										//
	int rcv_n=0;									//��M������

	UCHAR delim;									//��M�f�[�^�f���~�^
	UCHAR rcv_buff[MAX_BUFF][MAX_LENGTH];	//��M�����O�o�b�t�@
	UCHAR stmp[MAX_LENGTH];						//
	int com_ok=0;
	UCHAR CommRcvBuff[256];
	UCHAR CommSendBuff[1024];
	UCHAR SendBuff[512];
	unsigned char rbuff[MAX_LENGTH];
	unsigned char ucBCC;

	struct termios tio;									//�|�[�g�ݒ�\����

	// Member function
    bool init();
	void App_Init(void);
	void App_Close(void);
	int GetRcv_to_Cmd( char *rcv, char *prm);
	ULONG SendData(UCHAR *pucInput, USHORT usSize);
	void GetProductInfo(void);
	void SerialStart(void);
	void SerialStop(void);
	int Comm_Open(const char *dev);
	void Comm_Setup(long baud ,int parity ,int bitlen ,int rts ,int dtr ,char code);
	int Comm_SendData( UCHAR *buff, int l);
	int Comm_GetRcvData(UCHAR *buff);
	int Comm_CheckRcv();
	void Comm_Close();
	void Comm_Rcv(void);
};
