#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <device/driver.h>
#include <device/request.h>
#include <device/class/valio.h>
#include <device/valio/touchpad.h>
#include <device/class/i2c.h>
#include <device/class/icu.h>
#include <device/irq.h>

#define dprintk(x...) do {} while (0)

#define MTCH6102_POINT_PER_CELL ((uint32_t)64)

#define MTCH6102_REG_FWMAJOR 0x00
#define MTCH6102_REG_FWMINOR 0x01
#define MTCH6102_REG_APPIDH 0x02
#define MTCH6102_REG_APPIDL 0x03
#define MTCH6102_REG_CMD 0x04
#define MTCH6102_REG_MODE 0x05
#define MTCH6102_REG_MODECON 0x06

#define MTCH6102_REG_TOUCHSTATE 0x10
#define MTCH6102_REG_TOUCHX 0x11
#define MTCH6102_REG_TOUCHY 0x12
#define MTCH6102_REG_TOUCHLSB 0x13
#define MTCH6102_REG_GESTURESTATE 0x14
#define MTCH6102_REG_GESTUREDIAG 0x15

#define MTCH6102_REG_NUMBEROFXCHANNELS 0x20
#define MTCH6102_REG_NUMBEROFYCHANNELS 0x21
#define MTCH6102_REG_SCANCOUNT 0x22
#define MTCH6102_REG_TOUCHTHRESHX 0x23
#define MTCH6102_REG_TOUCHTHRESHY 0x24
#define MTCH6102_REG_ACTIVEPERIODL 0x25
#define MTCH6102_REG_ACTIVEPERIODH 0x26
#define MTCH6102_REG_IDLEPERIODL 0x27
#define MTCH6102_REG_IDLEPERIODH 0x28
#define MTCH6102_REG_IDLETIMEOUT 0x29
#define MTCH6102_REG_HYSTERESIS 0x2A
#define MTCH6102_REG_DEBOUNCEUP 0x2B
#define MTCH6102_REG_DEBOUNCEDOWN 0x2C
#define MTCH6102_REG_BASEINTERVALL 0x2D
#define MTCH6102_REG_BASEINTERVALH 0x2E
#define MTCH6102_REG_BASEPOSFILTER 0x2F
#define MTCH6102_REG_BASENEGFILTER 0x30
#define MTCH6102_REG_FILTERTYPE 0x31
#define MTCH6102_REG_FILTERSTRENGTH 0x32
#define MTCH6102_REG_BASEFILTERTYPE 0x33
#define MTCH6102_REG_BASEFILTERSTRENGTH 0x34
#define MTCH6102_REG_LARGEACTIVATIONTHRESHL 0x35
#define MTCH6102_REG_LARGEACTIVATIONTHRESHH 0x36
#define MTCH6102_REG_HORIZONTALSWIPEDISTANCE 0x37
#define MTCH6102_REG_VERTICALSWIPEDISTANCE 0x38
#define MTCH6102_REG_SWIPEHOLDBOUNDARY 0x39
#define MTCH6102_REG_TAPDISTANCE 0x3A
#define MTCH6102_REG_DISTANCEBETWEENTAPS 0x3B
#define MTCH6102_REG_TAPHOLDTIMEL 0x3C
#define MTCH6102_REG_TAPHOLDTIMEH 0x3D
#define MTCH6102_REG_GESTURECLICKTIME 0x3E
#define MTCH6102_REG_SWIPEHOLDTHRESH 0x3F
#define MTCH6102_REG_MINSWIPEVELOCITY 0x40
#define MTCH6102_REG_HORIZONTALGESTUREANGLE 0x41
#define MTCH6102_REG_VERTICALGESTUREANGLE 0x42
#define MTCH6102_REG_I2CADDR 0x43

GCT_CONTAINER_FCNS(dev_request_queue, static, dev_request_queue,
                   remove, push);

DRIVER_PV(struct mtch6102_priv_s
{
    struct valio_touchpad_state_s last_state;
    struct dev_i2c_rq_s i2c_req;
    struct dev_i2c_transfer_s i2c_transfer[2];
    uint8_t wdata[4];
    uint8_t rdata[4];
    struct device_i2c_s i2c;
    uint8_t saddr;
    struct dev_irq_src_s irq;
    struct dev_valio_rq_s *pending;
    uint8_t width, height;

    dev_request_queue_root_t queue;
});

static void mtch6102_request_run(
    struct device_s *dev,
    struct mtch6102_priv_s *pv,
    bool_t from_irq);

static KROUTINE_EXEC(mtch6102_mode_done)
{
    struct mtch6102_priv_s *pv;
    struct device_s *dev;

    dprintk("%s\n", __FUNCTION__);

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.pvdata;

    LOCK_SPIN_IRQ(&dev->lock);

    pv->pending = NULL;
    mtch6102_request_run(dev, pv, 0);

    LOCK_RELEASE_IRQ(&dev->lock);
}

static KROUTINE_EXEC(mtch6102_channels_done)
{
    struct mtch6102_priv_s *pv;
    struct device_s *dev;

    dprintk("%s\n", __FUNCTION__);

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.pvdata;

    LOCK_SPIN_IRQ(&dev->lock);

    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, mtch6102_mode_done);

    pv->i2c_req.pvdata = dev;
    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_req.saddr = pv->saddr;
    pv->i2c_transfer[0].data = pv->wdata;
    pv->i2c_transfer[0].size = 2;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_req.transfer_count = 1;

    pv->wdata[0] = MTCH6102_REG_MODE;
    pv->wdata[1] = 2;

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);

    LOCK_RELEASE_IRQ(&dev->lock);
}

static void mtch6102_configure(
    struct device_s *dev,
    struct mtch6102_priv_s *pv)
{
    /* Hack to hold requests while configuring */
    pv->pending = (void*)1;

    dprintk("%s\n", __FUNCTION__);

    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, mtch6102_channels_done);

    pv->i2c_req.pvdata = dev;
    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_req.saddr = pv->saddr;
    pv->i2c_transfer[0].data = pv->wdata;
    pv->i2c_transfer[0].size = 3;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_req.transfer_count = 1;

    pv->wdata[0] = MTCH6102_REG_NUMBEROFXCHANNELS;
    pv->wdata[1] = pv->width;
    pv->wdata[2] = pv->height;

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static KROUTINE_EXEC(mtch6102_state_done)
{
    struct mtch6102_priv_s *pv;
    struct valio_touchpad_state_s st;
    struct dev_valio_rq_s *rq = NULL;
    struct device_s *dev;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.pvdata;

    LOCK_SPIN_IRQ(&dev->lock);

    rq = pv->pending;
    pv->pending = NULL;

    dprintk("%s pv %p dev %p rq %p\n", __FUNCTION__, pv, dev, rq);

    if (!rq) {
        struct dev_request_s *drq = dev_request_queue_head(&pv->queue);
        if (drq)
            rq = dev_valio_rq_s_cast(drq);
    }

    st.touch = pv->rdata[0] & 0x1;
    st.x = ((uint16_t)pv->rdata[1] << 4) | (pv->rdata[3] >> 4);
    st.y = ((uint16_t)pv->rdata[2] << 4) | (pv->rdata[3] & 0xf);

    if (rq && (rq->type == DEVICE_VALIO_READ
               || st.touch != pv->last_state.touch
               || (st.touch && (st.x != pv->last_state.x
                                || st.y != pv->last_state.y)))) {
        dev_valio_rq_remove(&pv->queue, rq);
        memcpy(rq->data, &st, sizeof(st));
        rq->error = 0;

        pv->last_state = st;
        dev_valio_rq_done(rq);
    } else {
        // Consider change was not big enough for WAIT_UPDATE to
        // succeed.
        rq = NULL;
    }

    mtch6102_request_run(dev, pv, 0);
    LOCK_RELEASE_IRQ(&dev->lock);
}

static void mtch6102_request_run(
    struct device_s *dev,
    struct mtch6102_priv_s *pv,
    bool_t from_irq)
{
    bool_t empty = dev_rq_queue_isempty(&pv->queue);

    dprintk("%s %d %d %s\n", __FUNCTION__, from_irq, pv->pending,
           empty ? "empty" : "rq");

    if (empty || pv->pending)
        return;

    struct dev_request_s *drq = dev_request_queue_head(&pv->queue);

    dprintk("%s %p\n", __FUNCTION__, drq);

    assert(drq);

    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(drq);

    dprintk("%s %p %d\n", __FUNCTION__, rq, rq->type);

    if (!from_irq && rq->type == DEVICE_VALIO_WAIT_EVENT) {
        dprintk("%s wait update\n", __FUNCTION__);
        return;
    }

    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, mtch6102_state_done);

    assert(rq->attribute == VALIO_TOUCHPAD_STATE);

    pv->i2c_req.pvdata = dev;
    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_req.saddr = pv->saddr;
    pv->i2c_transfer[0].data = pv->wdata;
    pv->i2c_transfer[0].size = 1;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_transfer[1].data = pv->rdata;
    pv->i2c_transfer[1].size = 4;
    pv->i2c_transfer[1].type = DEV_I2C_READ;
    pv->i2c_req.transfer_count = 2;

    pv->wdata[0] = MTCH6102_REG_TOUCHSTATE;

    dprintk("%s i2c op\n", __FUNCTION__);

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static DEV_VALIO_REQUEST(mtch6102_request)
{
    struct device_s *dev = accessor->dev;
    struct mtch6102_priv_s *pv = dev->drv_pv;
    error_t err = -EINVAL;

    dprintk("%s\n", __FUNCTION__);

    LOCK_SPIN_IRQ(&dev->lock);

    switch (rq->type) {
    case DEVICE_VALIO_READ:
        switch (rq->attribute) {
        case VALIO_TOUCHPAD_SIZE:
            ((struct valio_touchpad_size_s *)rq->data)->width
                = pv->width * MTCH6102_POINT_PER_CELL;
            ((struct valio_touchpad_size_s *)rq->data)->height
                = pv->height * MTCH6102_POINT_PER_CELL;
            err = 0;
            break;

        case VALIO_TOUCHPAD_STATE:
            dev_valio_rq_pushback(&pv->queue, req);
            err = 1;
            break;
        }

        break;

    case DEVICE_VALIO_WAIT_EVENT:
        switch (rq->attribute) {
        case VALIO_TOUCHPAD_STATE:
            dev_valio_rq_pushback(&pv->queue, req);
            err = 1;
            break;

        default:
            err = -EINVAL;
            break;
        }
    }

    if (err > 0)
        mtch6102_request_run(dev, pv, 0);

    LOCK_RELEASE_IRQ(&dev->lock);

    if (err <= 0) {
        rq->error = err;
        
        dev_valio_rq_done(req);
    }
}

static DEV_IRQ_SRC_PROCESS(mtch6102_irq)
{
    struct device_s *dev = ep->base.dev;
    struct mtch6102_priv_s *pv = dev->drv_pv;

    dprintk("%s\n", __FUNCTION__);

    lock_spin(&dev->lock);

    mtch6102_request_run(dev, pv, 1);

    lock_release(&dev->lock);
}


#define mtch6102_use dev_use_generic
#define mtch6102_cancel (dev_valio_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(mtch6102_init)
{
    struct mtch6102_priv_s *pv;
    uintptr_t width, height;


    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    if (!pv)
        return -ENOMEM;

    memset(pv, 0, sizeof(*pv));

    if (device_get_param_dev_accessor(dev, "bus", &pv->i2c, DRIVER_CLASS_I2C)) {
        printk("Bus not found\n");
        goto err_pv;
    }

    if (dev_i2c_res_get_addr(dev, &pv->saddr, 0)) {
        printk("Address not found\n");
        goto err_pv;
    }

    if (device_get_param_uint(dev, "width", &width)) {
        printk("Size not found\n");
        goto err_pv;
    }

    if (device_get_param_uint(dev, "height", &height)) {
        printk("Size not found\n");
        goto err_pv;
    }

    pv->width = width;
    pv->height = height;

    device_irq_source_init(
        dev, &pv->irq, 1,
        &mtch6102_irq /*, DEV_IRQ_SENSE_LOW_LEVEL*/);

    if (device_irq_source_link(dev, &pv->irq, 1, -1))
        goto err_pv;

    dev_rq_queue_init(&pv->queue);

    dev->drv_pv = pv;

    mtch6102_configure(dev, pv);

    return 0;

  err_pv:
    mem_free(pv);

    return -1;
}

static DEV_CLEANUP(mtch6102_cleanup)
{
    struct mtch6102_priv_s *pv = dev->drv_pv;

    device_put_accessor(&pv->i2c.base);
    dev_rq_queue_destroy(&pv->queue);
    mem_free(pv);
}

DRIVER_DECLARE(mtch6102_drv, 0, "MTCH6102 Touchpad", mtch6102,
               DRIVER_VALIO_METHODS(mtch6102));

DRIVER_REGISTER(mtch6102_drv);

