// SPDX-License-Identifier: GPL-2.0
// dma_proxy.c — AXI DMA S2MM with synchronized multi-channel start and data gate
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Microchip12");
MODULE_DESCRIPTION("DMA proxy: S2MM with synchronized start and data gate control");

// -------- IOCTLs --------
#define START_XFER         0x1001
#define FINISH_XFER        0x1002
#define GET_PROD_IDX       0x2001
#define RESET_PROD_IDX     0x2002
#define GET_STATS          0x2004
#define RESET_DMA          0x3001
#define START_ALL_CHANNELS 0x3002

// -------- AXI DMA S2MM registers --------
#define S2MM_CONTROL_REGISTER       0x30
#define S2MM_STATUS_REGISTER        0x34
#define S2MM_DST_ADDRESS_REGISTER   0x48
#define S2MM_DST_ADDRESS_MSB        0x4C
#define S2MM_BUFF_LENGTH_REGISTER   0x58

#define S2MM_CTRL_RUN    0x00000001
#define S2MM_CTRL_RESET  0x00000004

#define STATUS_HALTED           0x00000001
#define STATUS_IDLE             0x00000002
#define STATUS_DMA_INTERNAL_ERR 0x00000010
#define STATUS_DMA_SLAVE_ERR    0x00000020
#define STATUS_DMA_DECODE_ERR   0x00000040
#define STATUS_IOC_IRQ          0x00001000
#define STATUS_DLY_IRQ          0x00002000
#define STATUS_ERR_IRQ          0x00004000
#define STATUS_IRQ_MASK        (STATUS_IOC_IRQ | STATUS_DLY_IRQ | STATUS_ERR_IRQ)

struct dma_proxy_stats {
    u32 phys_lo;
    u32 last_da_lo;
    u32 last_dmasr;
    u32 prod_idx;
};

struct dma_proxy_dev {
    struct device *dev;
    void __iomem  *regs;

    void       *buf;
    dma_addr_t  phys;
    size_t      frame_bytes;
    u32         buffer_count;
    size_t      buf_size;

    u32 last_da_lo;
    u32 last_dmasr;

    u32 prod_idx;
    u32 pending;

    struct cdev cdev;
    dev_t devt;
    
    u32 instance_id;
};

static struct class *dma_proxy_class;

// -------- Global channel registry for synchronized start --------
#define MAX_DMA_CHANNELS 3
static struct dma_proxy_dev *g_dma_channels[MAX_DMA_CHANNELS] = {NULL, NULL, NULL};
static DEFINE_SPINLOCK(g_channel_lock);
static int g_num_channels = 0;

// -------- Module parameters (MUST be declared before use) --------
static u32 dma_debug = 0;
module_param_named(debug, dma_debug, uint, 0644);
MODULE_PARM_DESC(debug, "Verbose logs (0=off,1=on)");

static u32 param_buffer_count = 2;
module_param_named(buffer_count, param_buffer_count, uint, 0444);
MODULE_PARM_DESC(buffer_count, "Frames in buffer");

static u32 param_frame_bytes = 33554432;
module_param_named(frame_bytes, param_frame_bytes, uint, 0444);
MODULE_PARM_DESC(frame_bytes, "Bytes per frame (BTT)");

static u32 param_data_gate_addr = 0x41200000;
module_param_named(data_gate_addr, param_data_gate_addr, uint, 0444);
MODULE_PARM_DESC(data_gate_addr, "Physical address of data gate control register");

// -------- Data gate control (MMIO) --------
static void __iomem *g_data_gate_reg = NULL;
static phys_addr_t g_data_gate_addr = 0x41200000;

static inline void data_gate_enable(bool enable)
{
    if (g_data_gate_reg) {
        iowrite32(enable ? 0x1 : 0x0, g_data_gate_reg);
        wmb();
        if (dma_debug)
            pr_info("dma_proxy: data_gate %s\n", enable ? "ENABLED" : "DISABLED");
    }
}

// -------- helpers --------
static inline void wr(struct dma_proxy_dev *d, u32 off, u32 val)
{ iowrite32(val, d->regs + off); wmb(); }

static inline u32 rd(struct dma_proxy_dev *d, u32 off)
{ u32 v = ioread32(d->regs + off); rmb(); return v; }

static void s2mm_soft_reset(struct dma_proxy_dev *d)
{
    unsigned long deadline;
    
    wr(d, S2MM_CONTROL_REGISTER, 0);
    udelay(2);
    wr(d, S2MM_STATUS_REGISTER, 0xFFFFFFFF);
    wmb();
    wr(d, S2MM_DST_ADDRESS_REGISTER, 0);
    wr(d, S2MM_BUFF_LENGTH_REGISTER, 0);
    wmb();
    wr(d, S2MM_CONTROL_REGISTER, S2MM_CTRL_RESET);
    wmb();

    deadline = jiffies + msecs_to_jiffies(100);
    while (time_before(jiffies, deadline)) {
        if (!(rd(d, S2MM_CONTROL_REGISTER) & S2MM_CTRL_RESET))
            break;
        cpu_relax();
        udelay(1);
    }

    wr(d, S2MM_STATUS_REGISTER, 0xFFFFFFFF);
    wmb();
}

static int axi_dma_reset(struct dma_proxy_dev *d)
{
    unsigned long deadline;
    
    wr(d, S2MM_CONTROL_REGISTER, 0);
    udelay(2);
    wr(d, S2MM_STATUS_REGISTER, 0xFFFFFFFF);
    wmb();
    wr(d, S2MM_DST_ADDRESS_REGISTER, 0);
    wr(d, S2MM_BUFF_LENGTH_REGISTER, 0);
    wmb();
    wr(d, S2MM_CONTROL_REGISTER, S2MM_CTRL_RESET);
    wmb();

    deadline = jiffies + msecs_to_jiffies(100);
    while (time_before(jiffies, deadline)) {
        if (!(rd(d, S2MM_CONTROL_REGISTER) & S2MM_CTRL_RESET)) {
            wr(d, S2MM_STATUS_REGISTER, 0xFFFFFFFF);
            wmb();
            return 0;
        }
        cpu_relax();
        udelay(1);
    }

    dev_err(d->dev, "AXI DMA S2MM reset timeout\n");
    return -ETIMEDOUT;
}

static dma_addr_t frame_dma_addr(struct dma_proxy_dev *d, u32 frame_idx)
{
    return d->phys + frame_idx * d->frame_bytes;
}

static int dma_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct dma_proxy_dev *d = file->private_data;
    size_t size = vma->vm_end - vma->vm_start;
    if (size > d->buf_size)
        return -EINVAL;
    return dma_mmap_coherent(d->dev, vma, d->buf, d->phys, size);
}

static void kick_s2mm_frame(struct dma_proxy_dev *d, u32 frame_idx)
{
    dma_addr_t da = frame_dma_addr(d, frame_idx);
    wr(d, S2MM_STATUS_REGISTER, STATUS_IRQ_MASK);
    wr(d, S2MM_DST_ADDRESS_REGISTER, (u32)da);
    wr(d, S2MM_CONTROL_REGISTER, S2MM_CTRL_RUN);
    wr(d, S2MM_BUFF_LENGTH_REGISTER, (u32)d->frame_bytes);

    d->last_da_lo = (u32)da;
    if (dma_debug)
        dev_info(d->dev, "kick idx=%u DA=0x%08x LEN=%u\n",
                 frame_idx, d->last_da_lo, (u32)d->frame_bytes);
}

// -------- IOCTLs --------
static long dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct dma_proxy_dev *d = file->private_data;
    u32 n, st;
    unsigned long deadline, flags;
    int i, started;
    struct dma_proxy_dev *ch;
    
    switch (cmd) {
    case START_XFER:
        n = (u32)arg;
        if (n == 0 || n > d->buffer_count) return -EINVAL;
        if (d->pending) return -EBUSY;
        if (d->frame_bytes & 0x3) return -EINVAL;

        d->pending = n;
        kick_s2mm_frame(d, d->prod_idx);
        return 0;

    case FINISH_XFER:
        while (d->pending) {
            deadline = jiffies + msecs_to_jiffies(60000);
            do {
                st = rd(d, S2MM_STATUS_REGISTER);
                d->last_dmasr = st;

                if (st & (STATUS_DMA_INTERNAL_ERR | STATUS_DMA_SLAVE_ERR | STATUS_DMA_DECODE_ERR)) {
                    dev_err(d->dev, "S2MM ERR DMASR=0x%08x\n", st);
                    wr(d, S2MM_STATUS_REGISTER, STATUS_ERR_IRQ);
                    s2mm_soft_reset(d);
                    d->pending = 0;
                    return -EIO;
                }

                if ((st & STATUS_IDLE) && (st & STATUS_IOC_IRQ)) {
                    wr(d, S2MM_STATUS_REGISTER, STATUS_IOC_IRQ);
                    d->prod_idx = (d->prod_idx + 1) % d->buffer_count;
                    d->pending--;
                    if (d->pending)
                        kick_s2mm_frame(d, d->prod_idx);
                    goto frame_done;
                }

                cpu_relax();
                udelay(1);
            } while (time_before(jiffies, deadline));

            dev_err(d->dev, "S2MM timeout: DMASR=0x%08x\n", st);
            s2mm_soft_reset(d);
            d->pending = 0;
            return -ETIMEDOUT;

frame_done:
            ;
        }
        return 0;

    case GET_PROD_IDX: {
        u32 idx = d->prod_idx;
        if (copy_to_user((void __user *)arg, &idx, sizeof(idx)))
            return -EFAULT;
        return 0;
    }

    case RESET_PROD_IDX:
        d->prod_idx = 0;
        return 0;

    case GET_STATS: {
        struct dma_proxy_stats s;
        s.phys_lo = (u32)d->phys;
        s.last_da_lo = d->last_da_lo;
        s.last_dmasr = d->last_dmasr;
        s.prod_idx = d->prod_idx;
        if (copy_to_user((void __user *)arg, &s, sizeof(s)))
            return -EFAULT;
        return 0;
    }
    
    case RESET_DMA:
        return axi_dma_reset(d);

    case START_ALL_CHANNELS:
        started = 0;
        
        spin_lock_irqsave(&g_channel_lock, flags);
        
        // Step 1: Disable data gate (blocks PRBS -> FIFO)
        data_gate_enable(false);
        udelay(10);
        
        // Step 2: Validate all channels are ready
        for (i = 0; i < MAX_DMA_CHANNELS; i++) {
            ch = g_dma_channels[i];
            if (!ch) continue;
            
            if (ch->pending) {
                spin_unlock_irqrestore(&g_channel_lock, flags);
                data_gate_enable(true);
                dev_err(d->dev, "Channel %d busy, cannot start all\n", i);
                return -EBUSY;
            }
            
            if (ch->frame_bytes & 0x3) {
                spin_unlock_irqrestore(&g_channel_lock, flags);
                data_gate_enable(true);
                dev_err(d->dev, "Channel %d invalid frame_bytes\n", i);
                return -EINVAL;
            }
        }
        
        // Step 3: Start all DMAs (they wait for data since gate is disabled)
        for (i = 0; i < MAX_DMA_CHANNELS; i++) {
            ch = g_dma_channels[i];
            if (!ch) continue;
            
            ch->pending = 1;
            kick_s2mm_frame(ch, ch->prod_idx);
            started++;
        }
        
        wmb();
        
        // Step 4: Enable data gate (all DMAs now receive data simultaneously)
        data_gate_enable(true);
        
        spin_unlock_irqrestore(&g_channel_lock, flags);
        
        if (dma_debug || started > 0) {
            dev_info(d->dev, "START_ALL_CHANNELS: kicked %d channels with data gate sync\n", started);
        }
        
        return started > 0 ? 0 : -ENODEV;

    default:
        return -ENOTTY;
    }
}

static int dma_open(struct inode *inode, struct file *file)
{
    struct dma_proxy_dev *d = container_of(inode->i_cdev, struct dma_proxy_dev, cdev);
    file->private_data = d;
    return 0;
}

static int dma_release(struct inode *inode, struct file *file) { return 0; }

static const struct file_operations fops = {
    .owner          = THIS_MODULE,
    .open           = dma_open,
    .release        = dma_release,
    .unlocked_ioctl = dma_ioctl,
    .mmap           = dma_mmap,
};

// -------- Channel registration --------
static int register_dma_channel(struct dma_proxy_dev *d, u32 instance_id)
{
    unsigned long flags;
    
    if (instance_id >= MAX_DMA_CHANNELS) {
        dev_err(d->dev, "instance_id %u exceeds MAX_DMA_CHANNELS\n", instance_id);
        return -EINVAL;
    }
    
    spin_lock_irqsave(&g_channel_lock, flags);
    
    if (g_dma_channels[instance_id]) {
        spin_unlock_irqrestore(&g_channel_lock, flags);
        dev_err(d->dev, "Channel %u already registered\n", instance_id);
        return -EEXIST;
    }
    
    g_dma_channels[instance_id] = d;
    g_num_channels++;
    d->instance_id = instance_id;
    
    spin_unlock_irqrestore(&g_channel_lock, flags);
    
    dev_info(d->dev, "Registered as channel %u (%d/%d active)\n", 
             instance_id, g_num_channels, MAX_DMA_CHANNELS);
    
    return 0;
}

static void unregister_dma_channel(struct dma_proxy_dev *d)
{
    unsigned long flags;
    
    spin_lock_irqsave(&g_channel_lock, flags);
    
    if (d->instance_id < MAX_DMA_CHANNELS && g_dma_channels[d->instance_id] == d) {
        g_dma_channels[d->instance_id] = NULL;
        g_num_channels--;
        dev_info(d->dev, "Unregistered channel %u\n", d->instance_id);
    }
    
    spin_unlock_irqrestore(&g_channel_lock, flags);
}

// -------- probe / remove --------
static int dma_proxy_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    struct of_phandle_args dma_spec;
    struct resource res;
    struct dma_proxy_dev *d;
    int rc;
    u32 index = 0;
    char dev_name[32];

    if (!np) return -ENODEV;

    if (of_property_read_u32(np, "instance", &index)) {
        dev_err(dev, "Missing 'instance' property in device tree\n");
        return -EINVAL;
    }
    
    if (index >= MAX_DMA_CHANNELS) {
        dev_err(dev, "instance %u exceeds MAX_DMA_CHANNELS (%d)\n", 
                index, MAX_DMA_CHANNELS);
        return -EINVAL;
    }

    d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
    if (!d) return -ENOMEM;
    d->dev = dev;

    d->frame_bytes  = param_frame_bytes;
    d->buffer_count = param_buffer_count;
    if (!d->frame_bytes || !d->buffer_count || (d->frame_bytes & 0x3))
        return -EINVAL;

    rc = of_parse_phandle_with_args(np, "dmas", "#dma-cells", 0, &dma_spec);
    if (rc) {
        dev_err(dev, "failed to parse 'dmas' phandle (%d)\n", rc);
        return rc;
    }

    rc = of_address_to_resource(dma_spec.np, 0, &res);
    of_node_put(dma_spec.np);
    if (rc) return rc;

    d->regs = devm_ioremap_resource(dev, &res);
    if (IS_ERR(d->regs)) return PTR_ERR(d->regs);

    rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
    if (rc) return rc;

    d->buf_size = d->frame_bytes * d->buffer_count;
    d->buf = dmam_alloc_coherent(dev, d->buf_size, &d->phys, GFP_KERNEL);
    if (!d->buf) {
        dev_err(dev, "CMA alloc %zu bytes failed\n", d->buf_size);
        return -ENOMEM;
    }

    d->prod_idx = 0;
    d->pending  = 0;

    rc = alloc_chrdev_region(&d->devt, index, 1, "dma_proxy");
    if (rc) return rc;

    cdev_init(&d->cdev, &fops);
    d->cdev.owner = THIS_MODULE;
    rc = cdev_add(&d->cdev, d->devt, 1);
    if (rc) goto err_chr;

    if (!dma_proxy_class) {
        dma_proxy_class = class_create(THIS_MODULE, "dma_proxy_class");
        if (IS_ERR(dma_proxy_class)) {
            rc = PTR_ERR(dma_proxy_class);
            dma_proxy_class = NULL;
            goto err_cdev;
        }
    }

    snprintf(dev_name, sizeof(dev_name), "dma_proxy%u", index);
    if (!device_create(dma_proxy_class, NULL, d->devt, NULL, dev_name)) {
        rc = -ENODEV;
        goto err_class;
    }

    rc = register_dma_channel(d, index);
    if (rc) {
        device_destroy(dma_proxy_class, d->devt);
        goto err_class;
    }

    dev_info(dev, "dma_proxy ready: ring=%u x %zu bytes, /dev/%s\n",
             d->buffer_count, d->frame_bytes, dev_name);

    platform_set_drvdata(pdev, d);
    return 0;

err_class:
    if (dma_proxy_class) { class_destroy(dma_proxy_class); dma_proxy_class = NULL; }
err_cdev:
    cdev_del(&d->cdev);
err_chr:
    unregister_chrdev_region(d->devt, 1);
    return rc;
}

static int dma_proxy_remove(struct platform_device *pdev)
{
    struct dma_proxy_dev *d = platform_get_drvdata(pdev);

    if (!d) return 0;

    unregister_dma_channel(d);
    
    device_destroy(dma_proxy_class, d->devt);
    cdev_del(&d->cdev);
    unregister_chrdev_region(d->devt, 1);
    if (d->buf)
        dma_free_coherent(d->dev, d->buf_size, d->buf, d->phys);

    return 0;
}

static const struct of_device_id dt_ids[] = {
    { .compatible = "xlnx,dma_proxy" },
    { }
};
MODULE_DEVICE_TABLE(of, dt_ids);

static struct platform_driver dma_proxy_pdrv = {
    .driver = {
        .name = "dma_proxy",
        .of_match_table = dt_ids,
    },
    .probe  = dma_proxy_probe,
    .remove = dma_proxy_remove,
};

static int __init dma_proxy_init(void)
{
    int ret;
    
    g_data_gate_addr = param_data_gate_addr;
    if (g_data_gate_addr) {
        g_data_gate_reg = ioremap(g_data_gate_addr, 0x1000);
        if (!g_data_gate_reg) {
            pr_err("dma_proxy: Failed to map data_gate register at 0x%08x\n",
                   (u32)g_data_gate_addr);
            pr_info("dma_proxy: Continuing without data gate control\n");
        } else {
            pr_info("dma_proxy: Data gate control mapped at 0x%08x\n",
                    (u32)g_data_gate_addr);
            data_gate_enable(false);
        }
    }
    
    ret = platform_driver_register(&dma_proxy_pdrv);
    if (ret) {
        if (g_data_gate_reg) {
            iounmap(g_data_gate_reg);
            g_data_gate_reg = NULL;
        }
    }
    
    return ret;
}

static void __exit dma_proxy_exit(void)
{
    if (g_data_gate_reg) {
        data_gate_enable(false);
        iounmap(g_data_gate_reg);
        g_data_gate_reg = NULL;
    }
    
    platform_driver_unregister(&dma_proxy_pdrv);
}

module_init(dma_proxy_init);
module_exit(dma_proxy_exit);
