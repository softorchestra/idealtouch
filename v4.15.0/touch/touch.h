#ifndef __TOUCH_H__
#define __TOUCH_H__

#define   IOCTL_IDEALTOUCH_FLUSH_BUFFER     _IO('U', 0x01)
#define   IOCTL_IDEALTOUCH_WRITEBUFFER      _IOW('U', 0x02, InBuffer)
#define   IOCTL_IDEALTOUCH_READLENGTH       _IOR('U', 0x03, int)
#define   IOCTL_IDEALTOUCH_READBUFFER       _IOWR('U', 0x04, InBuffer)
#define   IOCTL_IDEALTOUCH_CALIREADY        _IOR('U', 0x05, CaliReady)
#define   IOCTL_IDEALTOUCH_CALSETTING       _IOW('U', 0x06, unsigned char)
#define   IOCTL_IDEALTOUCH_PENUP            _IOR('U', 0x07, unsigned char)
#define   IOCTL_IDEALTOUCH_3P_CALSETTING    _IOW('U', 0x08, unsigned char)

#define   MAX_INBUFFER                      10
#define   MAX_OUTBUFFER                     128


typedef  struct _INBUFFER
{
    unsigned char      Buf[MAX_INBUFFER];
    unsigned short     Len;
} InBuffer, *pInBuffer;

typedef  struct _OUTBUFFER
{
    unsigned char      Buf[MAX_OUTBUFFER];
    unsigned short     Len;
} OutBuffer, *pOutBuffer;

typedef  struct _CALIREADY
{
    int   Avg_Point[2];
    unsigned char     Ready;
} CaliReady, *pCaliReady;

#endif
