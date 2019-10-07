/*
 *  Native support for the Idealtek USB Touch Panel
 *
 *  Copyright (c) 2004      IdealTEK <ideal.j491108@msa.hinet.net>
 *
 *  based on aiptek.c by
 *           Bryan W. Headley
 *           Chris Atenasio
 *  ChangeLog:
 *      v1.1 - Base on Aiptek Hypen USB Driver
 *
 * NOTE:
 *      This kernel driver is augmented by the "Idealtek" XFree86 input
 *      driver for your X server, as well as a GUI Front-end "IdealTouch Utility".
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/proc_fs.h>
#include <linux/fs.h> 
#include <asm/uaccess.h>
#include <linux/poll.h>
#include "touch.h"

/*********************************************************************************************
 * Version Information
 **********************************************************************************************/
#define DRIVER_VERSION "v1.1 Aug-24-2004"
#define DRIVER_AUTHOR  "Chang-Hsieh Wu <ideal.j491108@msa.hinet.net>"
#define DRIVER_DESC    "Idealtek USB Touch Panel Driver (Linux 2.5.x)"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
#define usb_alloc_coherent(a, b, c, d) usb_buffer_alloc(a, b, c, d)
#define usb_free_coherent(a, b, c, d) usb_buffer_free(a, b, c, d)
#endif


#define  MAX_RESOLUTION      0x3FFF
#define  TIMEOUT             15  /* timeout = 1.5 seconds */
/* timeout =if (usb_submit_urb(wacom->irq, GFP_KERNEL))
	return -EIO; 1.5 seconds */

/**********************************************************************************************
 * idealtek data packet:
 * (returned as Report 1)
 *
 *        bit7  bit6  bit5  bit4  bit3  bit2  bit1  bit0
 * byte0  sync   tch   0     0     0     0     0    rclk
 * byte1  0      X6    X5    X4    X3    X2    X1    X0
 * byte2  0      X13   X12   X11   X10   X9    X8    X78
 * byte3  0      Y6    Y5    Y4    Y3    Y2    Y1    Y0
 * byte4  0      Y13   Y12   Y11   Y10   Y9    Y8    Y7
 *
 *  The procfs interface
 *  --------------------
 *
 *  This driver supports delivering configuration/status reports
 *  through {procfs}/driver/usb/idealtek. ("procfs" is normally mounted
 *  to /proc.) Said file can be found while the driver is active in
 *  memory; it will be removed when the driver is removed, either
 *  through user intervention (rmmod idealtek) or through software
 *  such as "hotplug".
 *
 **********************************************************************************************/


#define IDEALTEK_MINOR           70
#define SAMPLES                  60
#define REPORT_MAX               4096

struct idealtek_features
{
    char *name;
    int  packet_len;
    int  x_max;
    int  y_max;
    usb_complete_t irq;   /*void (*irq) (struct urb *urb);*/
};

struct idealtek
{
    signed char *data;
    dma_addr_t data_dma;
    unsigned char      rawData[5];

    unsigned char      ReportData[REPORT_MAX];
    unsigned short     InRIndex;
    unsigned short     OutRIndex;

    unsigned char      QueueData[128];
    unsigned char      InIndex;
    unsigned char      OutIndex;
    unsigned char      curByte;
    struct usb_device  *usbdev;
    struct urb         *irq;
    struct idealtek_features *features;
    unsigned int ifnum;
    int open_count;

    struct semaphore   mutex; /*for removing flag*/
    wait_queue_head_t  wait;
    wait_queue_head_t  remove_ok;
    int opened;
    int remove_pending;

    /* for calibration*/
    unsigned char CaliType;
    unsigned char CurrCaliPnt;
    unsigned char CalibrationDone;
    unsigned char PenUp;
    unsigned char DoingCalibration;
    int           AvgPoint[9][2];
    unsigned char Samples;

};

static struct idealtek *idealtek = NULL;
static int maxtime = TIMEOUT;
/*extern devfs_handle_t usb_devfs_handle;*/

/*======================================================================

========================================================================*/

static ssize_t idealtek_fops_write(struct file *file, const char *buf,
                                   size_t count, loff_t *ppos)
{
    return -ESPIPE;
}

static unsigned int idealtek_fops_poll(struct file *file, poll_table *wait)
{
    struct idealtek *s = file->private_data;

    poll_wait(file, &s->wait, wait);
    if (s->InRIndex != s->OutRIndex)
    {
        return POLLIN | POLLRDNORM;
    }
    return 0;
}

static ssize_t idealtek_fops_read(struct file *file, char *buf,
                                  size_t count, loff_t *ppos)
{
    int  bytes_to_read, Len, i;
    unsigned char buffer[128];
    struct idealtek *s = (struct idealtek *) file->private_data;
    int  ret;

    // printk("copy_to_user, %d, %d\n",s->InRIndex,s->OutRIndex);
    if (ppos != &file->f_pos)
    {
        //     printk("ppos =%ld file->f_pos=%ld\n",ppos,&file->f_pos);
        //return -ESPIPE;
    }

    if (s->remove_pending)
    {
        // printk("s->remove_pending\n");
        return -EIO;
    }
    if (!s->usbdev)
    {
        //   printk("s->remove_pending\n");
        return -EIO;
    }

    if (count > 128)
    {
        count = 128;
    }

    if (s->InRIndex != s->OutRIndex)
    {
        if (s->OutRIndex > s->InRIndex)
        {
            Len = REPORT_MAX - s->OutRIndex + s->InRIndex;
        }
        else
        {
            Len = s->InRIndex - s->OutRIndex;
        }

        if (Len > count)
        {
            bytes_to_read = count;
        }
        else
        {
            bytes_to_read = Len;
        }


        for (i = 0; i < bytes_to_read; i++)
        {
            buffer[i] = s->ReportData[s->OutRIndex++];
            if (s->OutRIndex >= REPORT_MAX)
            {
                s->OutRIndex = 0;
            }
        }
    }
    else
    {
        return 0;
    }
    // printk("idealtek_fops_read end.\n");
    if (copy_to_user(buf, buffer, bytes_to_read))
    {
        ret = -EFAULT;
    }
    else
    {
        ret = bytes_to_read;
    }

    return ret;
}


static loff_t idealtek_fops_lseek(struct file *file, loff_t offset,
                                  int origin)
{
    return -ESPIPE;
}


static int idealtek_fops_open(struct inode *inode, struct file *file)
{
    unsigned m = iminor(inode);
    int retval;

    printk("idealtek_fops_open==>minor: %d\n", m);

    /* if (m != IDEALTEK_MINOR) return -EINVAL;*/

    /*retval = -ENODEV;*/
    if (!idealtek || !idealtek->usbdev)
    {
        printk("idealtek instance is not exist\n");
        return -EINVAL;
    }

    /* Mark the device as opened */

    idealtek->opened++;
    file->private_data = idealtek;
    retval = 0;

    return retval;
}

static int idealtek_fops_release(struct inode *inode, struct file *file)
{
    struct idealtek *s = (struct idealtek *) file->private_data;

    /* Close device */
    //  printk("idealtek_fops_release==>Enter\n");

    down(&s->mutex);
    s->opened --;
    if (s->opened < 0)
    {
        s->opened = 0;
    }
    up(&s->mutex);
    //  printk("idealtek_fops_release==>End\n");

    return 0;
}


/* Issue a specific vendor command */
int vendor_command(struct usb_device *usbdev, char *buf, __u16 Len)
{
    int retval;
    unsigned int pipe;

    /*printk("Data out: Len=%d ", Len);
    for (i=0; i<Len; i++) printk(" %02X", buf[i]);
    printk("\n");*/

    pipe = usb_sndctrlpipe(usbdev, 0);  /* upstream */
    retval = usb_control_msg(usbdev, pipe, 0,
                             USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                             0, 0, buf, Len,
                             HZ / (maxtime / 10)); //100

    return retval;
}


void QueueResponseData(unsigned char theChar, struct idealtek *idealtek)
{
    idealtek->QueueData[idealtek->InIndex++] = theChar;
    if (idealtek->InIndex >= 128)
    {
        idealtek->InIndex = 0;
    }
}

int GetResponseData(struct idealtek *s, unsigned char *buf, int Len)
{
    int i;
    for (i = 0; i < Len; i++)
    {
        if (s->InIndex == s->OutIndex)
        {
            break;
        }
        else
        {
            buf[i] = s->QueueData[s->OutIndex++];
            if (s->OutIndex >= 128)
            {
                s->OutIndex = 0;
            }
        }
    }
    return i;
}


int GetResponseDataLength(struct idealtek *s)
{
    int  Length = 0;
    if (s->OutIndex > s->InIndex)
    {
        Length = (128 - s->OutIndex) + s->InIndex;
    }
    else
    {
        Length = s->InIndex - s->OutIndex;
    }
    return Length;
}


void FlushResponseData(struct idealtek *s)
{
    memset(s->QueueData, 0, sizeof(s->QueueData));
    s->OutIndex = 0;
    s->InIndex = 0;
}

static int idealtek_fops_ioctl(struct file *file,
                               unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct idealtek *s = (struct idealtek *) file->private_data;

    if (s->remove_pending)
    {
        return -EIO;
    }

    if (!s->usbdev)
    {
        up(&s->mutex);
        return -EIO;
    }

    switch (cmd)
    {
        case 0:
            break;
        case IOCTL_IDEALTOUCH_WRITEBUFFER:
            {
                InBuffer   inbuff;

                /*printk("IOCTL_IDEALTOUCH_WRITEBUFFER\n");*/
                if (copy_from_user(&inbuff, (void *)arg, sizeof(InBuffer)))
                {
                    return -EFAULT;
                }

                if (inbuff.Len <= MAX_INBUFFER)
                {
                    vendor_command(s->usbdev, (char *)inbuff.Buf, (__u16)inbuff.Len);
                }
                else
                {
                    return -EFAULT;
                }
            }
            ret = 0;
            break;
        case IOCTL_IDEALTOUCH_READBUFFER:
            {
                OutBuffer   outbuff;
                if (copy_from_user(&outbuff, (void *)arg, sizeof(OutBuffer)))
                {
                    printk("IOCTL_IDEALTOUCH_READBUFFER: copy from user error arg.");
                    return -EFAULT;
                }

                if (outbuff.Len < 128)
                {
                    outbuff.Len = (unsigned short) GetResponseData(s, outbuff.Buf, outbuff.Len);
                }
                else
                {
                    return -EFAULT;
                }

                if (copy_to_user((void *)arg, &outbuff, sizeof(OutBuffer)))
                {
                    return -EFAULT;
                }
                ret = 0;
            }
            break;
        case IOCTL_IDEALTOUCH_READLENGTH:
            {
                int  i;

                i = GetResponseDataLength(s);
                if (copy_to_user((void *)arg, &i, sizeof(int)))
                {
                    return -EFAULT;
                }
                ret = 0;
            }
            break;
        case IOCTL_IDEALTOUCH_FLUSH_BUFFER:
            FlushResponseData(s);
            ret = 0;
            break;
        case IOCTL_IDEALTOUCH_CALIREADY:
            {
                CaliReady  cReady;
                if (s->CalibrationDone == 1)
                {
                    cReady.Avg_Point[0] = s->AvgPoint[s->CurrCaliPnt][0];
                    cReady.Avg_Point[1] = s->AvgPoint[s->CurrCaliPnt][1];
                    cReady.Ready = 1;
                }
                else
                {
                    cReady.Ready = 0;
                    cReady.Avg_Point[0] = 0;
                    cReady.Avg_Point[1] = 0;
                }

                if (copy_to_user((void *)arg, &cReady, sizeof(cReady)))
                {
                    return -EFAULT;
                }

                ret = 0;
            }
            break;
        case IOCTL_IDEALTOUCH_CALSETTING:
            {
                unsigned char nowPoint;

                if (copy_from_user(&nowPoint, (void *)arg, sizeof(nowPoint)))
                {
                    printk("IOCTL_IDEALTOUCH_CALSETTING. Error");
                    return -EFAULT;
                }
                printk("IOCTL_IDEALTOUCH_CALSETTING: %d\n", nowPoint);
                s->CaliType = 59;
                s->CurrCaliPnt = nowPoint;
                s->CalibrationDone = 0;
                s->PenUp = 0;
                if (s->CurrCaliPnt == 0xFF)
                {
                    s->DoingCalibration = 0;
                }
                else
                {
                    s->DoingCalibration = 1;
                }
                s->AvgPoint[s->CurrCaliPnt][0] = 0;
                s->AvgPoint[s->CurrCaliPnt][1] = 0;
                s->Samples = 0; //Global Variable
                ret = 0;
            }
            break;
        case IOCTL_IDEALTOUCH_3P_CALSETTING:
            {
                unsigned char nowPoint;

                if (copy_from_user(&nowPoint, (void *)arg, sizeof(nowPoint)))
                {
                    printk("IOCTL_IDEALTOUCH_3P_CALSETTING. Error");
                    return -EFAULT;
                }
                printk("IOCTL_IDEALTOUCH_3P_CALSETTING: %d\n", nowPoint);
                s->CaliType = 3;
                s->CurrCaliPnt = nowPoint;
                s->CalibrationDone = 0;
                s->PenUp = 0;
                if (s->CurrCaliPnt > 3)
                {
                    s->DoingCalibration = 0;
                }
                else
                {
                    s->DoingCalibration = 1;
                }
                s->AvgPoint[s->CurrCaliPnt][0] = 0;
                s->AvgPoint[s->CurrCaliPnt][1] = 0;
                s->Samples = 0; //Global Variable
                ret = 0;
            }
            break;
        case IOCTL_IDEALTOUCH_PENUP:
            {
                unsigned char  PenUp;

                if (s->PenUp == 1)
                {
                    PenUp = 1;
                }
                else
                {
                    PenUp = 0;
                }

                if (copy_to_user((void *)arg, &PenUp, sizeof(PenUp)))
                {
                    return -EFAULT;
                }
                ret = 0;
            }

            break;
        default:
            ret = -ENOIOCTLCMD;
            break;
    }

    up(&s->mutex);
    return ret;
}

static struct file_operations idealtek_fops =
{
    .owner = THIS_MODULE,
    .llseek = idealtek_fops_lseek,
    .read = idealtek_fops_read,
    .write = idealtek_fops_write,
    .unlocked_ioctl = idealtek_fops_ioctl,
    .open = idealtek_fops_open,
    .poll = idealtek_fops_poll,
    .release = idealtek_fops_release,
};

/*======================================================================
Function Name :
    GetCaliRawData
Description:
     Caculate Calibration Data
Author:
     Chang-Hsieh Wu, 2003-10-15
Comment:
     None
======================================================================*/

void GetCaliRawData(struct idealtek *s)
{
    int            CXY[2];
    unsigned char  fButtonDown = 0;
    unsigned char  SignBit;
    unsigned char  SignByte;

    if (s->rawData[0] & 0x40)
    {
        fButtonDown = 1;
    }

    if (s->CaliType != 3)
    {
        if ((s->Samples <= SAMPLES) && (fButtonDown == 1))  /*Pen Down and Get data*/
        {
            if (s->Samples > (SAMPLES / 4) && s->Samples <= (SAMPLES * 3 / 4))
            {
                SignBit = 0; /* X sign*/
                if (s->rawData[0] & 0x10)
                {
                    SignByte = 2;
                }
                else
                {
                    SignByte = 1;
                }

                if (s->rawData[SignByte] & 0x40)
                {
                    SignBit = 1;
                    s->rawData[SignByte] = s->rawData[SignByte] & 0x3F;
                }

                if (s->rawData[0] & 0x10)
                {
                    CXY[0] = (((int)s->rawData[1]) >> 3) + (((int)s->rawData[2]) << 4);    /*X*/
                }
                else
                {
                    CXY[0] = (((int)s->rawData[2]) >> 3) + (((int)s->rawData[1]) << 4);    /*X*/
                }

                if (SignBit == 1)
                {
                    CXY[0] = CXY[0] - 1024;
                }
                CXY[0] = (1024 + CXY[0]) << 1;
                s->AvgPoint[s->CurrCaliPnt][0] += CXY[0];

                SignBit = 0; /*Y sign*/
                if (s->rawData[0] & 0x10)
                {
                    SignByte = 4;
                }
                else
                {
                    SignByte = 3;
                }

                if (s->rawData[SignByte] & 0x40)
                {
                    SignBit = 1;
                    s->rawData[SignByte] = s->rawData[SignByte] & 0x3F;
                }
                if (s->rawData[0] & 0x10)
                {
                    CXY[1] = (((int)s->rawData[3]) >> 3) + (((int)s->rawData[4]) << 4);    //Y
                }
                else
                {
                    CXY[1] = (((int)s->rawData[4]) >> 3) + (((int)s->rawData[3]) << 4);    //Y
                }

                if (SignBit == 1)
                {
                    CXY[1] = CXY[1] - 1024;
                }
                CXY[1] = (1024 + CXY[1]) << 1;
                s->AvgPoint[s->CurrCaliPnt][1] += CXY[1];
            }

            s->Samples++;
            if (s->Samples == (SAMPLES * 3 / 4 + 1))
            {
                s->AvgPoint[s->CurrCaliPnt][0] = s->AvgPoint[s->CurrCaliPnt][0] / (SAMPLES / 2);
                s->AvgPoint[s->CurrCaliPnt][1] = s->AvgPoint[s->CurrCaliPnt][1] / (SAMPLES / 2);
            }

        }
        else if (s->Samples < SAMPLES && fButtonDown == 0) /*Error Touch: Reset RawDataBuffer*/
        {
            s->Samples = 0;
            s->AvgPoint[s->CurrCaliPnt][0] = 0;
            s->AvgPoint[s->CurrCaliPnt][1] = 0;
        }
        else if (s->Samples > SAMPLES) /*Finished*/
        {
            s->CalibrationDone = 1; /*Inform Api that had finished calibration.*/
            s->Samples++;
            if (s->Samples > (SAMPLES + 10))
            {
                s->Samples = SAMPLES + 10;
            }
            if (fButtonDown == 0)  /*Pen Up*/
            {
                s->DoingCalibration = 0; /**/
                s->Samples = 0;
                s->PenUp = 1;
            }
        }
    }
    else  /*3 pnts calibration */
    {
        if ((s->Samples <= SAMPLES) && (fButtonDown == 1))  /*Pen Down and Get data*/
        {
            if (s->Samples > (SAMPLES / 4) && s->Samples <= (SAMPLES * 3 / 4))
            {

                if (s->rawData[0] & 0x10)
                {
                    CXY[0] = (((int)s->rawData[1]) >> 2) + (((int)s->rawData[2]) << 5); /*X*/
                    CXY[1] = (((int)s->rawData[3]) >> 2) + (((int)s->rawData[4]) << 5); /*Y*/
                }
                else
                {
                    CXY[0] = (((int)s->rawData[2]) >> 2) + (((int)s->rawData[1]) << 5); /*X*/
                    CXY[1] = (((int)s->rawData[4]) >> 2) + (((int)s->rawData[3]) << 5); /*Y*/
                }

                s->AvgPoint[s->CurrCaliPnt][0] += CXY[0];
                s->AvgPoint[s->CurrCaliPnt][1] += CXY[1];
            }

            if (s->Samples == (SAMPLES * 3 / 4 + 1))
            {
                s->AvgPoint[s->CurrCaliPnt][0] = s->AvgPoint[s->CurrCaliPnt][0] / (SAMPLES / 2);
                s->AvgPoint[s->CurrCaliPnt][1] = s->AvgPoint[s->CurrCaliPnt][1] / (SAMPLES / 2);
            }

            s->Samples++;
        }
        else if (s->Samples < SAMPLES && fButtonDown == 0) /*Error Touch: Reset RawDataBuffer*/
        {
            s->Samples = 0;
            s->AvgPoint[s->CurrCaliPnt][0] = 0;
            s->AvgPoint[s->CurrCaliPnt][1] = 0;
        }
        else if (s->Samples > SAMPLES) /*Finished*/
        {
            s->CalibrationDone = 1; /*Inform Api that had finished calibration.*/
            s->Samples++;
            if (s->Samples > (SAMPLES + 10))
            {
                s->Samples = SAMPLES + 10;
            }
            if (fButtonDown == 0)  /*Pen Up*/
            {
                s->DoingCalibration = 0; /**/
                s->Samples = 0;
                s->PenUp = 1;
            }
        }
    }
}

/*======================================================================
Function Name :
    ReportCoord
Description:
     Report the coordinate
Author:
     Chang-Hsieh Wu, 2003-2-28
Comment:
     None
======================================================================*/

static void
QReportData(struct idealtek *s)
{
    unsigned char i;

    for (i = 0; i < 5 ; i++)
    {
        s->ReportData[s->InRIndex++] = s->rawData[i];
        if (s->InRIndex >= REPORT_MAX)
        {
            s->InRIndex = 0;
        }
    }
    // printk("QReportData, %d, %d\n",s->InRIndex,s->OutRIndex);
}


static void
idealtek_irq(struct urb *urb, struct pt_regs *regs)
{
    struct idealtek *s = urb->context;
    unsigned char *data = s->data;
    int Len, i, retval;

    /* printk("irq start\n"); */
    switch (urb->status)
    {
        case 0:
            /* success */
            break;
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
            /* this urb is terminated, clean up */
            printk("%s - urb shutting down with status: %d", __FUNCTION__, urb->status);
            return;
        default:
            printk("%s - nonzero urb status received: %d", __FUNCTION__, urb->status);
            goto exit;
    }

    /*idealtek->eventCount++;*/
    /* printk("Data In: Len=%d ", urb->actual_length);
       for (Len=0; Len<8; Len++) {
          printk("%02X ", data[Len]);
          if (data[Len]==0xFF) break;
           }
       printk("\n");
    */
    /*for (Len=0; Len<8; Len++)
        if (data[Len]==0xFF) break;*/
    Len = urb->actual_length;
    // printk("copy_to_user, %d, %d\n",s->InRIndex,s->OutRIndex);
    for (i = 0; i < Len; i++)
    {
        s->rawData[s->curByte] = data[ i ];
        switch (s->curByte)
        {
            case 0:  // First byte has the sync bit set, next two don't.
                if ((s->rawData[0] & 0x80) != 0)
                {
                    s->curByte++;
                }
                else
                {
                    if (s->rawData[0] != 0x00)
                    {
                        QueueResponseData(s->rawData[0], s);
                    }
                }
                break;
            case 1:
                if ((s->rawData[1] & 0x80) == 0)
                {
                    s->curByte++;
                }
                else
                {
                    s->rawData[0] = s->rawData[1];
                    s->curByte = 1;
                }
                break;
            case 2:
                if ((s->rawData[2] & 0x80) == 0)
                {
                    s->curByte++;
                }
                else
                {
                    s->rawData[0] = s->rawData[2];
                    s->curByte = 1;
                }
                break;
            case 3:
                if ((s->rawData[3] & 0x80) == 0)
                {
                    s->curByte++;
                }
                else
                {
                    s->rawData[0] = s->rawData[3];
                    s->curByte = 1;
                }
                break; printk("\n");
            case 4:
                if ((s->rawData[4] & 0x80) == 0)
                {
                    if (s->DoingCalibration == 1)
                    {
                        GetCaliRawData(s);
                    }
                    else
                    {
                        QReportData(s);
                    }
                    s->curByte = 0;
                }
                else
                {
                    s->rawData[0] = s->rawData[4];
                    s->curByte = 1;
                }
        }
    }

    memset(s->data, 0xFF, sizeof(s->data));

exit:
    retval = usb_submit_urb(urb, GFP_ATOMIC);

    if (retval != 0)
    {
        printk("irq end\n");
    }

    if (waitqueue_active(&s->wait))
    {
        wake_up_interruptible(&s->wait);
    }
    return;
}

struct idealtek_features idealtek_features[] =
{
    { "Idealtek", 8, 16383, 16383, idealtek_irq },
    { NULL, 0}
};


struct usb_device_id idealtek_ids[] =
{
    { USB_DEVICE(0x8888, 0x6666), driver_info: 0 },
    { USB_DEVICE(0x1391, 0x1000), driver_info: 0 },
    { USB_DEVICE(0x1391, 0x2000), driver_info: 0 },
    {}
};

MODULE_DEVICE_TABLE(usb, idealtek_ids);


static struct usb_class_driver touch_class =
{
    .name =		"idtk%d",
    .fops =		&idealtek_fops,
    //	.mode =		S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
    .minor_base =	IDEALTEK_MINOR,
};


static int
idealtek_probe(struct usb_interface *intf,
               const struct usb_device_id *id)
{
    struct usb_device *dev = interface_to_usbdev(intf);
    /*struct usb_host_interface *interface = intf->altsetting + 0;*/
    struct usb_endpoint_descriptor *endpoint;
    int retval;

    //printk("idealtek_probe Start\n");
    retval = usb_register_dev(intf, &touch_class);
    if (retval)
    {
        printk("unable to get a minor for this device");
        printk("Not able to get a minor for this device.");
        return -ENOMEM;
    }

    if (!(idealtek = kmalloc(sizeof(struct idealtek), GFP_KERNEL)))
    {
        return -ENOMEM;
    }

    memset(idealtek, 0, sizeof(struct idealtek));

    idealtek->data = usb_alloc_coherent(dev, 10, GFP_ATOMIC, &idealtek->data_dma);
    if (!idealtek->data)
    {
        kfree(idealtek);
        return -ENOMEM;
    }

    idealtek->curByte = 0;
    idealtek->InIndex = 0;
    idealtek->OutIndex = 0;
    sema_init(&idealtek->mutex, 1);
    
    idealtek->usbdev = NULL;
    idealtek->opened = 0;
    idealtek->remove_pending = 0;
    init_waitqueue_head(&idealtek->wait);
    init_waitqueue_head(&idealtek->remove_ok);

    idealtek->irq = usb_alloc_urb(0, GFP_KERNEL);
    if (!idealtek->irq)
    {
        usb_free_coherent(dev, 10, idealtek->data, idealtek->data_dma);
        kfree(idealtek);
        return -ENOMEM;
    }

    /*This used to be meaningful, when we had a matrix of
    different models with statically-assigned different
    features. Now we ask the tablet about everything.*/

    idealtek->features = idealtek_features;

    /* Reset the tablet. The tablet boots up in 'SwitchtoMouse'
       mode, which indicates relative coordinates. 'SwitchToTablet'
       infers absolute coordinates. (Ergo, mice are inferred to be
       relative-only devices, which is not true. A misnomer.)
       The routine we use, aiptek_program_tablet, has been generalized
       enough such that it's callable through the procfs interface.
       This is why we use struct aiptek throughout.*/
    idealtek->usbdev        = dev;

    endpoint = &intf->altsetting[0].endpoint[0].desc;
    /*usb_set_idle(dev, dev->config[0].interface[ifnum].altsetting[0].bInterfaceNumber, 0, 0);*/
    if (idealtek->features->packet_len > 10)
    {
        BUG();
    }

    usb_fill_int_urb(idealtek->irq, dev,
                     usb_rcvintpipe(dev, endpoint->bEndpointAddress),
                     idealtek->data, idealtek->features->packet_len,
                     idealtek->features->irq, idealtek, endpoint->bInterval);
    idealtek->irq->transfer_dma = idealtek->data_dma;
    idealtek->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

    usb_set_intfdata(intf, idealtek);

    /*submit a urb*/
    usb_submit_urb(idealtek->irq, GFP_KERNEL);
    return 0;
}


static void
idealtek_disconnect(struct usb_interface *intf)
{
    struct idealtek *s  = usb_get_intfdata(intf);

    usb_set_intfdata(intf, NULL);
    if (s)
    {
        usb_deregister_dev(intf, &touch_class);
        usb_unlink_urb(s->irq);
        usb_free_urb(s->irq);
        usb_free_coherent(interface_to_usbdev(intf), 10, s->data, s->data_dma);
        kfree(s);
    }
}


static struct usb_driver idealtek_driver =
{
    //.owner =	THIS_MODULE,
    .name =		"idealtek",
    .probe =	idealtek_probe,
    .disconnect =	idealtek_disconnect,
    .id_table =	idealtek_ids,
};

static int __init
idealtek_init(void)
{
    int result;

    result = usb_register(&idealtek_driver);
    if (result == 0)
    {
        //modinfo(DRIVER_VERSION ": " DRIVER_AUTHOR);
        //modinfo(DRIVER_DESC);
    }
    return result;
}

static void __exit
idealtek_exit(void)
{
    usb_deregister(&idealtek_driver);
}

module_init(idealtek_init);
module_exit(idealtek_exit);
