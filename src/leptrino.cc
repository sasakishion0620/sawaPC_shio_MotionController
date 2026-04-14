#include "leptrino.h"

#ifndef LEPTRINO_DEVICE_PATH
#define LEPTRINO_DEVICE_PATH "/dev/ttyACM1"
#endif

// ----------------------------------------------------------------------------------
//	�A�v���P�[�V����������
// ----------------------------------------------------------------------------------
//	����	: non
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void leptrino::App_Init(void)
{
	int rt;
	const char *device_path = LEPTRINO_DEVICE_PATH;
	
	//Comm�|�[�g������
	com_ok = NG;
	rt = Comm_Open(device_path);
	if ( rt==OK ) {
		Comm_Setup( 460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
		com_ok = OK;
	}
	else {
		printf("ComPort Open Fail: %s\n", device_path);
	}

}

// ----------------------------------------------------------------------------------
//	�A�v���P�[�V�����I������
// ----------------------------------------------------------------------------------
//	����	: non
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void leptrino::App_Close(void)
{
	printf("Application Close\n");
	
	if ( com_ok == OK) {
		Comm_Close();
	}
}

/*********************************************************************************
* Function Name  : HST_SendResp
* Description    : �f�[�^�𐮌`���đ��M����
* Input          : pucInput ���M�f�[�^
*                : ���M�f�[�^�T�C�Y
* Output         : 
* Return         : 
*********************************************************************************/
ULONG leptrino::SendData(UCHAR *pucInput, USHORT usSize)
{
	USHORT usCnt;
	UCHAR ucWork;
	UCHAR ucBCC = 0;
	UCHAR *pucWrite = &CommSendBuff[0];
	USHORT usRealSize;
	
	// �f�[�^���` 
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_STX;					// STX 
	pucWrite++;
	usRealSize =2;
	
	for (usCnt = 0; usCnt < usSize; usCnt++) {
		ucWork = pucInput[usCnt];
		if (ucWork == CHR_DLE) {			// �f�[�^��0x10�Ȃ��0x10��t�� 
			*pucWrite = CHR_DLE;			// DLE�t�� 
			pucWrite++;						// �������ݐ� 
			usRealSize++;					// ���T�C�Y
			// BCC�͌v�Z���Ȃ�!
		}
		*pucWrite = ucWork;					// �f�[�^ 
		ucBCC ^= ucWork;					// BCC 
		pucWrite++;							// �������ݐ� 
		usRealSize++;						// ���T�C�Y 
	}
	
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_ETX;					// ETX 
	ucBCC ^= CHR_ETX;						// BCC�v�Z 
	pucWrite++;
	*pucWrite = ucBCC;						// BCC�t�� 
	usRealSize += 3;
	
	Comm_SendData(&CommSendBuff[0], usRealSize);
	
	return OK;
}

void leptrino::GetProductInfo(void)
{
	USHORT len;
	
	printf("Get SensorInfo\n");
	len = 0x04;								// �f�[�^��
	SendBuff[0] = len;						// �����O�X
	SendBuff[1] = 0xFF;						// �Z���TNo.
	SendBuff[2] = CMD_GET_INF;				// �R�}���h���
	SendBuff[3] = 0;						// �\��
	
	SendData(SendBuff, len);
}

void leptrino::SerialStart(void)
{
	USHORT len;
	
	printf("Start\n");
	len = 0x04;								// �f�[�^��
	SendBuff[0] = len;						// �����O�X
	SendBuff[1] = 0xFF;						// �Z���TNo.
	SendBuff[2] = CMD_DATA_START;			// �R�}���h���
	SendBuff[3] = 0;						// �\��
	
	SendData(SendBuff, len);
}

void leptrino::SerialStop(void)
{
	USHORT len;
	
	printf("Stop\n");
	len = 0x04;								// �f�[�^��
	SendBuff[0] = len;						// �����O�X
	SendBuff[1] = 0xFF;						// �Z���TNo.
	SendBuff[2] = CMD_DATA_STOP;			// �R�}���h���
	SendBuff[3] = 0;						// �\��
	
	SendData(SendBuff, len);
}

bool leptrino::init ()
{
    App_Init();
	
	if (com_ok == NG) {
		printf("ComPort Open Fail\n");
		return false;
	}
	
	// ���i���擾
	GetProductInfo();
	const int max_attempts = 1000;
	int attempts = 0;
	while(attempts < max_attempts) {
		attempts++;
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//��M�f�[�^�L
			CommRcvBuff[0]=0; 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
				stGetInfo->scFVer[F_VER_SIZE] = 0;
				printf("Version:%s\n", stGetInfo->scFVer);
				stGetInfo->scSerial[SERIAL_SIZE] = 0;
				printf("SerialNo:%s\n", stGetInfo->scSerial);
				stGetInfo->scPName[P_NAME_SIZE] = 0;
				printf("Type:%s\n", stGetInfo->scPName);
				printf("\n");
				EndF = 1;
			}
			
		}
		if ( EndF==1 ) break;
		usleep(1000);
	}

	if ( EndF != 1 ) {
		printf("Force sensor did not respond to product info request\n");
		App_Close();
		return false;
	}
	
	usleep(10000);

    SerialStart();
	EndF = 0;
    return true;
}

// ----------------------------------------------------------------------------------
//	�f�o�C�X�I�[�v��
// ----------------------------------------------------------------------------------
//	����	: dev .. �V���A���|�[�g
//	�߂�l	: ���펞:0   �G���[��:-1
// ----------------------------------------------------------------------------------
int leptrino::Comm_Open(const char *dev)
{
	//���ɃI�[�v�����Ă���Ƃ��͈�x����
	if (fd != 0) Comm_Close();
	//�|�[�g�I�[�v��
	fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd < 0) return NG;
	//�f���~�^
	delim=0;
		
	return OK;
}

// ----------------------------------------------------------------------------------
//	�f�o�C�X�N���[�Y
// ----------------------------------------------------------------------------------
//	����	: non
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void leptrino::Comm_Close()
{
	if (fd > 0) {
		close(fd);
	}
	fd=0;
		
	return;
}

// ----------------------------------------------------------------------------------
//	�|�[�g�ݒ�
// ----------------------------------------------------------------------------------
//	����	: boud   .. �{�[���[�g 9600 19200 ....
//			: parity .. �p���e�B�[ 
//			: bitlen .. �r�b�g��
//			: rts    .. RTS����
//			: dtr    .. DTR����
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void leptrino::Comm_Setup(long baud ,int parity ,int bitlen ,int rts ,int dtr ,char code)
{
	long brate;
	long cflg;
	
	switch (baud) {
	case 2400  :brate=B2400;  break;
	case 4800  :brate=B4800;  break;
	case 9600  :brate=B9600;  break;
	case 19200 :brate=B19200; break;
	case 38400 :brate=B38400; break;
	case 57600 :brate=B57600; break;
	case 115200:brate=B115200;break;
	case 230400:brate=B230400;break;
	case 460800:brate=B460800;break;
	default    :brate=B9600;  break;
	}
	//�p���e�B
	switch (parity) {
	case PAR_NON:cflg=0;					 break;
	case PAR_ODD:cflg=PARENB | PARODD;break;
	default     :cflg=PARENB;			 break;
	}
	//�f�[�^��
	switch (bitlen) {
	case 7 :cflg |= CS7;break;
	default:cflg |= CS8;break;
	}
	//DTR
	switch (dtr) {
	case 1 :cflg &= ~CLOCAL;break;
	default:cflg |= CLOCAL; break;
	}
	//RTS CTS
	switch (rts) {
	case 0 :cflg &= ~CRTSCTS;break;
	default:cflg |= CRTSCTS; break;
	}
	
	//�|�[�g�ݒ�t���O
	tio.c_cflag = cflg | CREAD;
	tio.c_lflag = 0;
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN]  = 0;
	
	cfsetspeed(&tio, brate);	
	tcflush( fd, TCIFLUSH);				//�o�b�t�@�̏���
	tcsetattr( fd, TCSANOW , &tio);		//�����̐ݒ�
	
	delim=code;								//�f���~�^�R�[�h
	return;
}

// ----------------------------------------------------------------------------------
//	�����񑗐M
// ----------------------------------------------------------------------------------
//	����	: buff .. ������o�b�t�@
//			: l    .. ���M������
//	�߂�l	: 1:OK -1:NG
// ----------------------------------------------------------------------------------
int leptrino::Comm_SendData( UCHAR *buff, int l)
{
	if (fd <= 0 ) return -1;
	
	write( fd, buff, l);
	
	return OK;
}

// ----------------------------------------------------------------------------------
//	��M�f�[�^�擾
// ----------------------------------------------------------------------------------
//	����	: buff .. ������o�b�t�@
//	�߂�l	: ��M������
// ----------------------------------------------------------------------------------
int leptrino::Comm_GetRcvData(UCHAR *buff)
{
	int l = rcv_buff[p_rd][0];
	const char *buff2 = (char*)(buff);
	
	if ( p_wr == p_rd ) return 0;
	
	memcpy(buff, &rcv_buff[p_rd][0], l);
	p_rd++;
	if (p_rd >= MAX_BUFF) p_rd=0;
	
	l=strlen(buff2);
	
	return l;
}

// ----------------------------------------------------------------------------------
//	��M�L���m�F
// ----------------------------------------------------------------------------------
//	����	: non 
//	�߂�l	: 0:�Ȃ� 0�ȊO�F����
// ----------------------------------------------------------------------------------
int leptrino::Comm_CheckRcv()
{
	return p_wr-p_rd;
}

// ----------------------------------------------------------------------------------
//	��M�Ď��X���b�h
// ----------------------------------------------------------------------------------
//	����	: pParam .. 
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void leptrino::Comm_Rcv(void)
{
	int i,rt=0;
	unsigned char ch;
	static int RcvSts = 0;

	while(1){
		rt=read(fd, rbuff, 1);
		//��M�f�[�^����
		if (rt > 0) {
			rbuff[rt]=0;
			ch=rbuff[0];
			
			switch (RcvSts) {
			case STS_IDLE:
				ucBCC = 0;								/* BCC */
				rcv_n = 0;
				if (ch == CHR_DLE) RcvSts = STS_WAIT_STX;
				break;
			case STS_WAIT_STX:
				if (ch == CHR_STX) {					/* STX������Ύ��̓f�[�^ */
					RcvSts = STS_DATA;
				} else {								/* STX�łȂ���Ό��ɖ߂� */
					RcvSts = STS_IDLE;
				}
				break;
			case STS_DATA:
				if (ch == CHR_DLE) {					/* DLE������Ύ���ETX */
					RcvSts = STS_WAIT_ETX;
				} else {								/* ��M�f�[�^�ۑ� */
					stmp[rcv_n] = ch;
					ucBCC ^= ch;						/* BCC */
					rcv_n++;
				}
				break;
			case STS_WAIT_ETX:
				if (ch == CHR_DLE) {					/* DLE�Ȃ�΃f�[�^�ł��� */
					stmp[rcv_n] = ch;
					ucBCC ^= ch;						/* BCC */
					rcv_n++;
					RcvSts = STS_DATA;
				} else if (ch == CHR_ETX) {				/* ETX�Ȃ玟��BCC */
					RcvSts = STS_WAIT_BCC;
					ucBCC ^= ch;						/* BCC */
				} else if (ch == CHR_STX) {			/* STX�Ȃ烊�Z�b�g */
					ucBCC = 0;							/* BCC */
					rcv_n = 0;
					RcvSts = STS_DATA;
				} else {
					ucBCC = 0;							/* BCC */
					rcv_n = 0;
					RcvSts = STS_IDLE;
				}
				break;
			case STS_WAIT_BCC:
				if (ucBCC == ch) {						/* BCC��v */
					//�쐬���ꂽ������������O�o�b�t�@�փR�s�[
					memcpy(rcv_buff[p_wr], stmp, rcv_n);
					p_wr++;
					if ( p_wr >= MAX_BUFF ) p_wr=0;
				}
				/* ���̃f�[�^��M�ɔ����� */
				ucBCC = 0;					/* BCC */
				rcv_n = 0;
				RcvSts = STS_IDLE;
				break;
			default:
				RcvSts = STS_IDLE;
				break;
			}
			
			if (rcv_n  > MAX_LENGTH) {
				ucBCC = 0;
				rcv_n = 0;
				RcvSts = STS_IDLE;
			}
		} else {
			break;
		}
		
		//��M�����t���O
		if (p_rd != p_wr) {
			Comm_RcvF=1;
		} else {
			Comm_RcvF=0;
		}
	}
}
