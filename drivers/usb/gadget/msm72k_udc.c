/*
 * Driver for HighSpeed USB Client Controller in MSM7K
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>

#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/pm_runtime.h>

#include <mach/msm72k_otg.h>
#include <linux/io.h>

#include <asm/mach-types.h>

#include <mach/board.h>
#include <mach/msm_hsusb.h>
#include <linux/device.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/clk.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>

static const char driver_name[] = "msm72k_udc";

/* #define DEBUG */
/* #define VERBOSE */

#define MSM_USB_BASE ((unsigned) ui->addr)

#define	DRIVER_DESC		"MSM 72K USB Peripheral Controller"
#define	DRIVER_NAME		"MSM72K_UDC"

#define EPT_FLAG_IN        0x0001

#define SETUP_BUF_SIZE     8


static const char *const ep_name[] = {
	"ep0out", "ep1out", "ep2out", "ep3out",
	"ep4out", "ep5out", "ep6out", "ep7out",
	"ep8out", "ep9out", "ep10out", "ep11out",
	"ep12out", "ep13out", "ep14out", "ep15out",
	"ep0in", "ep1in", "ep2in", "ep3in",
	"ep4in", "ep5in", "ep6in", "ep7in",
	"ep8in", "ep9in", "ep10in", "ep11in",
	"ep12in", "ep13in", "ep14in", "ep15in"
};

/*To release the wakelock from debugfs*/
static int release_wlocks;

struct msm_request {
	struct usb_request req;

	/* saved copy of req.complete */
	void	(*gadget_complete)(struct usb_ep *ep,
					struct usb_request *req);


	struct usb_info *ui;
	struct msm_request *next;
	struct msm_request *prev;

	unsigned busy:1;
	unsigned live:1;
	unsigned alloced:1;

	dma_addr_t dma;
	dma_addr_t item_dma;

	struct ept_queue_item *item;
};

#define to_msm_request(r) container_of(r, struct msm_request, req)
#define to_msm_endpoint(r) container_of(r, struct msm_endpoint, ep)
#define to_msm_otg(xceiv)  container_of(xceiv, struct msm_otg, otg)
#define is_b_sess_vld()	((OTGSC_BSV & readl(USB_OTGSC)) ? 1 : 0)
#define is_usb_online(ui) (ui->usb_state != USB_STATE_NOTATTACHED)

struct msm_endpoint {
	struct usb_ep ep;
	struct usb_info *ui;
	struct msm_request *req; /* head of pending requests */
	struct msm_request *last;
	unsigned flags;

	/* bit number (0-31) in various status registers
	** as well as the index into the usb_info's array
	** of all endpoints
	*/
	unsigned char bit;
	unsigned char num;

	unsigned wedged:1;
	/* pointers to DMA transfer list area */
	/* these are allocated from the usb_info dma space */
	struct ept_queue_head *head;
};

/* PHY status check timer to monitor phy stuck up on reset */
static struct timer_list phy_status_timer;

static void usb_do_work(struct work_struct *w);
static void usb_do_remote_wakeup(struct work_struct *w);


#define USB_STATE_IDLE    0
#define USB_STATE_ONLINE  1
#define USB_STATE_OFFLINE 2

#define USB_FLAG_START          0x0001
#define USB_FLAG_VBUS_ONLINE    0x0002
#define USB_FLAG_VBUS_OFFLINE   0x0004
#define USB_FLAG_RESET          0x0008
#define USB_FLAG_SUSPEND        0x0010
#define USB_FLAG_CONFIGURED     0x0020

#define USB_CHG_DET_DELAY	msecs_to_jiffies(1000)
#define REMOTE_WAKEUP_DELAY	msecs_to_jiffies(1000)
#define PHY_STATUS_CHECK_DELAY	(jiffies + msecs_to_jiffies(1000))

struct usb_info {
	/* lock for register/queue/device state changes */
	spinlock_t lock;

	/* single request used for handling setup transactions */
	struct usb_request *setup_req;

	struct platform_device *pdev;
	int irq;
	void *addr;

	unsigned state;
	unsigned flags;

	atomic_t configured;
	atomic_t running;

	struct dma_pool *pool;

	/* dma page to back the queue heads and items */
	unsigned char *buf;
	dma_addr_t dma;

	struct ept_queue_head *head;

	/* used for allocation */
	unsigned next_item;
	unsigned next_ifc_num;

	/* endpoints are ordered based on their status bits,
	** so they are OUT0, OUT1, ... OUT15, IN0, IN1, ... IN15
	*/
	struct msm_endpoint ept[32];


	/* max power requested by selected configuration */
	unsigned b_max_pow;
	unsigned chg_current;
	struct delayed_work chg_det;
	struct delayed_work chg_stop;
	struct msm_hsusb_gadget_platform_data *pdata;
	struct work_struct phy_status_check;

	struct work_struct work;
	unsigned phy_status;
	unsigned phy_fail_count;

	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct switch_dev sdev;

#define ep0out ept[0]
#define ep0in  ept[16]

	atomic_t ep0_dir;
	atomic_t test_mode;
	atomic_t offline_pending;
	atomic_t softconnect;
#ifdef CONFIG_USB_OTG
	u8 hnp_avail;
#endif

	atomic_t remote_wakeup;
	atomic_t self_powered;
	struct delayed_work rw_work;

	struct otg_transceiver *xceiv;
	enum usb_device_state usb_state;
	struct wake_lock	wlock;
};

static const struct usb_ep_ops msm72k_ep_ops;
static struct usb_info *the_usb_info;

static int msm72k_wakeup(struct usb_gadget *_gadget);
static int msm72k_pullup_internal(struct usb_gadget *_gadget, int is_active);
static int msm72k_set_halt(struct usb_ep *_ep, int value);
static void flush_endpoint(struct msm_endpoint *ept);
static void usb_reset(struct usb_info *ui);
static int usb_ept_set_halt(struct usb_ep *_ep, int value);

static void msm_hsusb_set_speed(struct usb_info *ui)
{
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	switch (readl(USB_PORTSC) & PORTSC_PSPD_MASK) {
	case PORTSC_PSPD_FS:
		dev_dbg(&ui->pdev->dev, "portchange USB_SPEED_FULL\n");
		ui->gadget.speed = USB_SPEED_FULL;
		break;
	case PORTSC_PSPD_LS:
		dev_dbg(&ui->pdev->dev, "portchange USB_SPEED_LOW\n");
		ui->gadget.speed = USB_SPEED_LOW;
		break;
	case PORTSC_PSPD_HS:
		dev_dbg(&ui->pdev->dev, "portchange USB_SPEED_HIGH\n");
		ui->gadget.speed = USB_SPEED_HIGH;
		break;
	}
	spin_unlock_irqrestore(&ui->lock, flags);
}

static void msm_hsusb_set_state(enum usb_device_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&the_usb_info->lock, flags);
	the_usb_info->usb_state = state;
	spin_unlock_irqrestore(&the_usb_info->lock, flags);
}

static enum usb_device_state msm_hsusb_get_state(void)
{
	unsigned long flags;
	enum usb_device_state state;

	spin_lock_irqsave(&the_usb_info->lock, flags);
	state = the_usb_info->usb_state;
	spin_unlock_irqrestore(&the_usb_info->lock, flags);

	return state;
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", sdev->state ? "online" : "offline");
}

static inline enum chg_type usb_get_chg_type(struct usb_info *ui)
{
	if ((readl(USB_PORTSC) & PORTSC_LS) == PORTSC_LS)
		return USB_CHG_TYPE__WALLCHARGER;
	else
		return USB_CHG_TYPE__SDP;
}

#define USB_WALLCHARGER_CHG_CURRENT 1800
static int usb_get_max_power(struct usb_info *ui)
{
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	unsigned long flags;
	enum chg_type temp;
	int suspended;
	int configured;
	unsigned bmaxpow;

	if (ui->gadget.is_a_peripheral)
		return -EINVAL;

	temp = atomic_read(&otg->chg_type);
	spin_lock_irqsave(&ui->lock, flags);
	suspended = ui->usb_state == USB_STATE_SUSPENDED ? 1 : 0;
	configured = atomic_read(&ui->configured);
	bmaxpow = ui->b_max_pow;
	spin_unlock_irqrestore(&ui->lock, flags);

	if (temp == USB_CHG_TYPE__INVALID)
		return -ENODEV;

	if (temp == USB_CHG_TYPE__WALLCHARGER)
		return USB_WALLCHARGER_CHG_CURRENT;

	if (suspended || !configured)
		return 0;

	return bmaxpow;
}

static int usb_phy_stuck_check(struct usb_info *ui)
{
	/*
	 * write some value (0xAA) into scratch reg (0x16) and read it back,
	 * If the read value is same as written value, means PHY is normal
	 * otherwise, PHY seems to have stuck.
	 */

	if (ui->xceiv->io_ops->write) {
		if (ui->xceiv->io_ops->write(ui->xceiv, 0xAA, 0x16) == -1) {
			dev_dbg(&ui->pdev->dev,
				"%s(): ulpi write timeout\n", __func__);
			return -EIO;
		}
	}
	if (ui->xceiv->io_ops->read) {
		if (ui->xceiv->io_ops->read(ui->xceiv, 0x16) != 0xAA) {
			dev_dbg(&ui->pdev->dev,
				"%s(): read value is incorrect\n", __func__);
			return -EIO;
		}
	}
	return 0;
}

/*
 * This function checks the phy status by reading/writing to the
 * phy scratch register. If the phy is stuck resets the HW
 * */
static void usb_phy_stuck_recover(struct work_struct *w)
{
	struct usb_info *ui = the_usb_info;
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	unsigned long flags;

	spin_lock_irqsave(&ui->lock, flags);
	if (ui->gadget.speed != USB_SPEED_UNKNOWN ||
			ui->usb_state == USB_STATE_NOTATTACHED ||
			ui->driver == NULL) {
		spin_unlock_irqrestore(&ui->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&ui->lock, flags);

	disable_irq(otg->irq);
	if (usb_phy_stuck_check(ui)) {
#ifdef CONFIG_USB_MSM_ACA
		del_timer_sync(&otg->id_timer);
#endif
		ui->phy_fail_count++;
		dev_err(&ui->pdev->dev,
				"%s():PHY stuck, resetting HW\n", __func__);
		/*
		 * PHY seems to have stuck,
		 * reset the PHY and HW link to recover the PHY
		 */
		usb_reset(ui);
#ifdef CONFIG_USB_MSM_ACA
		mod_timer(&otg->id_timer, jiffies +
				 msecs_to_jiffies(OTG_ID_POLL_MS));
#endif
		msm72k_pullup_internal(&ui->gadget, 1);
	}
	enable_irq(otg->irq);
}

static void usb_phy_status_check_timer(unsigned long data)
{
	struct usb_info *ui = the_usb_info;

	schedule_work(&ui->phy_status_check);
}

static void usb_chg_stop(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, chg_stop.work);
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	enum chg_type temp;

	temp = atomic_read(&otg->chg_type);

	if (temp == USB_CHG_TYPE__SDP)
		otg_set_power(ui->xceiv, 0);
}

static void usb_chg_detect(struct work_struct *w)
{
	struct usb_info *ui = container_of(w, struct usb_info, chg_det.work);
	struct msm_otg *otg = to_msm_otg(ui->xceiv);
	enum chg_type temp = USB_CHG_TYPE__INVALID;
	unsigned long flags;
	int maxpower;

	spin_lock_irqsave(&ui->lock, flags);
	if (ui->usb_state == USB_STATE_NOTATTACHED) {
		spin_unlock_irqrestore(&ui->lock, flags);
		return;
	}

	temp = usb_get_chg_type(ui);
	spin_unlock_irqrestore(&ui->lock, flags);

	atomic_set(&otg->chg_type, temp);
	maxpower = usb_get_max_power(ui);
	if (maxpower > 0)
		otg_set_power(ui->xceiv, maxpower);

	/* USB driver prevents idle and suspend power collapse(pc)
	 * while USB cable is connected. But when dedicated charger is
	 * connected, driver can vote for idle and suspend pc.
	 * OTG driver handles idle pc as part of above otg_set_power call
	 * when wallcharger is attached. To allow suspend pc, release the
	 * wakelock which will be re-acquired for any sub-sequent usb interrupts
	 * */
	if (temp == USB_CHG_TYPE__WALLCHARGER) {
		pm_runtime_put_sync(&ui->pdev->dev);
		wake_unlock(&ui->wlock);
	}
}

static int usb_ep_get_stall(struct msm_endpoint *ept)
{
	unsigned int n;
	struct usb_info *ui = ept->ui;

	n = readl(USB_ENDPTCTRL(ept->num));
	if (ept->flags & EPT_FLAG_IN)
		return (CTRL_TXS & n) ? 1 : 0;
	else
		return (CTRL_RXS & n) ? 1 : 0;
}

static void ulpi_write(struct usb_info *ui, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		;

	if (timeout == 0)
		dev_err(&ui->pdev->dev, "ulpi_write: timeout\n");
}

static void init_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++) {
		struct msm_endpoint *ept = ui->ept + n;

		ept->ui = ui;
		ept->bit = n;
		ept->num = n & 15;
		ept->ep.name = ep_name[n];
		ept->ep.ops = &msm72k_ep_ops;

		if (ept->bit > 15) {
			/* IN endpoint */
			ept->head = ui->head + (ept->num << 1) + 1;
			ept->flags = EPT_FLAG_IN;
		} else {
			/* OUT endpoint */
			ept->head = ui->head + (ept->num << 1);
			ept->flags = 0;
		}

	}
}

static void config_ept(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	unsigned cfg = CONFIG_MAX_PKT(ept->ep.maxpacket) | CONFIG_ZLT;

	/* ep0 out needs interrupt-on-setup */
	if (ept->bit == 0)
		cfg |= CONFIG_IOS;

	ept->head->config = cfg;
	ept->head->next = TERMINATE;

	if (ept->ep.maxpacket)
		dev_dbg(&ui->pdev->dev,
			"ept #%d %s max:%d head:%p bit:%d\n",
		       ept->num,
		       (ept->flags & EPT_FLAG_IN) ? "in" : "out",
		       ept->ep.maxpacket, ept->head, ept->bit);
}

static void configure_endpoints(struct usb_info *ui)
{
	unsigned n;

	for (n = 0; n < 32; n++)
		config_ept(ui->ept + n);
}

struct usb_request *usb_ept_alloc_req(struct msm_endpoint *ept,
			unsigned bufsize, gfp_t gfp_flags)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		goto fail1;

	req->item = dma_pool_alloc(ui->pool, gfp_flags, &req->item_dma);
	if (!req->item)
		goto fail2;

	if (bufsize) {
		req->req.buf = kmalloc(bufsize, gfp_flags);
		if (!req->req.buf)
			goto fail3;
		req->alloced = 1;
	}

	return &req->req;

fail3:
	dma_pool_free(ui->pool, req->item, req->item_dma);
fail2:
	kfree(req);
fail1:
	return 0;
}

static void usb_ept_enable(struct msm_endpoint *ept, int yes,
		unsigned char ep_type)
{
	struct usb_info *ui = ept->ui;
	int in = ept->flags & EPT_FLAG_IN;
	unsigned n;

	n = readl(USB_ENDPTCTRL(ept->num));

	if (in) {
		if (yes) {
			n = (n & (~CTRL_TXT_MASK)) |
				(ep_type << CTRL_TXT_EP_TYPE_SHIFT);
			n |= CTRL_TXE | CTRL_TXR;
		} else
			n &= (~CTRL_TXE);
	} else {
		if (yes) {
			n = (n & (~CTRL_RXT_MASK)) |
				(ep_type << CTRL_RXT_EP_TYPE_SHIFT);
			n |= CTRL_RXE | CTRL_RXR;
		} else
			n &= ~(CTRL_RXE);
	}
	/* complete all the updates to ept->head before enabling endpoint*/
	dma_coherent_pre_ops();
	writel(n, USB_ENDPTCTRL(ept->num));

	dev_dbg(&ui->pdev->dev, "ept %d %s %s\n",
	       ept->num, in ? "in" : "out", yes ? "enabled" : "disabled");
}

static void usb_ept_start(struct msm_endpoint *ept)
{
	struct usb_info *ui = ept->ui;
	struct msm_request *req = ept->req;
	int i, cnt;
	unsigned n = 1 << ept->bit;

	BUG_ON(req->live);

	while (req) {
		req->live = 1;
		/* prepare the transaction descriptor item for the hardware */
		req->item->info =
			INFO_BYTES(req->req.length) | INFO_IOC | INFO_ACTIVE;
		req->item->page0 = req->dma;
		req->item->page1 = (req->dma + 0x1000) & 0xfffff000;
		req->item->page2 = (req->dma + 0x2000) & 0xfffff000;
		req->item->page3 = (req->dma + 0x3000) & 0xfffff000;

		if (req->next == NULL) {
			req->item->next = TERMINATE;
			break;
		}
		req->item->next = req->next->item_dma;
		req = req->next;
	}

	/* link the hw queue head to the request's transaction item */
	ept->head->next = ept->req->item_dma;
	ept->head->info = 0;

	/* flush buffers before priming ept */
	dma_coherent_pre_ops();

	/* during high throughput testing it is observed that
	 * ept stat bit is not set even thoguh all the data
	 * structures are updated properly and ept prime bit
	 * is set. To workaround the issue, try to check if
	 * ept stat bit otherwise try to re-prime the ept
	 */
	for (i = 0; i < 5; i++) {
		writel(n, USB_ENDPTPRIME);
		for (cnt = 0; cnt < 3000; cnt++) {
			if (!(readl(USB_ENDPTPRIME) & n) &&
					(readl(USB_ENDPTSTAT) & n))
				return;
			udelay(1);
		}
	}

	if (!(readl(USB_ENDPTSTAT) & n))
		pr_err("Unable to prime the ept%d%s\n",
				ept->num,
				ept->flags & EPT_FLAG_IN ? "in" : "out");
}

int usb_ept_queue_xfer(struct msm_endpoint *ept, struct usb_request *_req)
{
	unsigned long flags;
	struct msm_request *req = to_msm_request(_req);
	struct msm_request *last;
	struct usb_info *ui = ept->ui;
	unsigned length = req->req.length;

	if (length > 0x4000)
		return -EMSGSIZE;

	spin_lock_irqsave(&ui->lock, flags);

	if (req->busy) {
		req->req.status = -EBUSY;
		spin_unlock_irqrestore(&ui->lock, flags);
		dev_err(&ui->pdev->dev,
			"usb_ept_queue_xfer() tried to queue busy request\n");
		return -EBUSY;
	}

	if (!atomic_read(&ui->configured) && (ept->num != 0)) {
		req->req.status = -ESHUTDOWN;
		spin_unlock_irqrestore(&ui->lock, flags);
		dev_err(&ui->pdev->dev,
			"usb_ept_queue_xfer() called while offline\n");
		return -ESHUTDOWN;
	}

	if (ui->usb_state == USB_STATE_SUSPENDED) {
		if (!atomic_read(&ui->remote_wakeup)) {
			req->req.status = -EAGAIN;
			spin_unlock_irqrestore(&ui->lock, flags);
			dev_err(&ui->pdev->dev,
				"%s: cannot queue as bus is suspended "
				"ept #%d %s max:%d head:%p bit:%d\n",
				__func__, ept->num,
				(ept->flags & EPT_FLAG_IN) ? "in" : "out",
				ept->ep.maxpacket, ept->head, ept->bit);

			return -EAGAIN;
		}

		wake_lock(&ui->wlock);
		otg_set_suspend(ui->xceiv, 0);
		schedule_delayed_work(&ui->rw_work, REMOTE_WAKEUP_DELAY);
	}

	req->busy = 1;
	req->live = 0;
	req->next = 0;
	req->req.status = -EBUSY;

	req->dma = dma_map_single(NULL, req->req.buf, length,
				  (ept->flags & EPT_FLAG_IN) ?
				  DMA_TO_DEVICE : DMA_FROM_DEVICE);


	/* Add the new request to the end of the queue */
	last = ept->last;
	if (last) {
		/* Already requests in the queue. add us to the
		 * end, but let the completion interrupt actually
		 * start things going, to avoid hw issues
		 */
		last->next = req;
		req->prev = last;

	} else {
		/* queue was empty -- kick the hardware */
		ept->req = req;
		req->prev = NULL;
		usb_ept_start(ept);
	}
	ept->last = req;

	spin_unlock_irqrestore(&ui->lock, flags);
	return 0;
}

/* --- endpoint 0 handling --- */

static void ep0_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_request *r = to_msm_request(req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;

	req->complete = r->gadget_complete;
	r->gadget_complete = 0;
	if	(req->complete)
		req->complete(&ui->ep0in.ep, req);
}

static void ep0_queue_ack_complete(struct usb_ep *ep,
	struct usb_request *_req)
{
	struct msm_request *r = to_msm_request(_req);
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	struct usb_request *req = ui->setup_req;

	/* queue up the receive of the ACK response from the host */
	if (_req->status == 0 && _req->actual == _req->length) {
		req->length = 0;
		if (atomic_read(&ui->ep0_dir) == USB_DIR_IN)
			usb_ept_queue_xfer(&ui->ep0out, req);
		else
			usb_ept_queue_xfer(&ui->ep0in, req);
		_req->complete = r->gadget_complete;
		r->gadget_complete = 0;
		if (_req->complete)
			_req->complete(&ui->ep0in.ep, _req);
	} else
		ep0_complete(ep, _req);
}

static void ep0_setup_ack_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct msm_endpoint *ept = to_msm_endpoint(ep);
	struct usb_info *ui = ept->ui;
	unsigned int temp;
	int test_mode = atomic_read(&ui->test_mode);

	if (!test_mode)
		return;

	switch (test_mode) {
	case J_TEST:
		dev_info(&ui->pdev->dev, "usb electrical test mode: (J)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_J_STATE, USB_PORTSC);
		break;

	case K_TEST:
		dev_info(&ui->pdev->dev, "usb electrical test mode: (K)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_K_STATE, USB_PORTSC);
		break;

	case SE0_NAK_TEST:
		dev_info(&ui->pdev->dev,
			"usb electrical test mode: (SE0-NAK)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_SE0_NAK, USB_PORTSC);
		break;

	case TST_PKT_TEST:
		dev_info(&ui->pdev->dev,
			"usb electrical test mode: (TEST_PKT)\n");
		temp = readl(USB_PORTSC) & (~PORTSC_PTC);
		writel(temp | PORTSC_PTC_TST_PKT, USB_PORTSC);
		break;
	}
}

static void ep0_setup_ack(struct usb_info *ui)
{
	struct usb_request *req = ui->setup_req;
	req->length = 0;
	req->complete = ep0_setup_ack_complete;
	usb_ept_queue_xfer(&ui->ep0in, req);
}

static void ep0_setup_stall(struct usb_info *ui
