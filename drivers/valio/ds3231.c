#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <device/driver.h>
#include <device/request.h>
#include <device/class/valio.h>
#include <device/valio/hwclock.h>
#include <device/class/i2c.h>

#define dprintk(x...) do {} while (0)

struct ds3231_regs {
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;

    // 7
    uint8_t a1_sec;
    uint8_t a1_min;
    uint8_t a1_hour;
    uint8_t a1_day_date;

    // b
    uint8_t a2_min;
    uint8_t a2_hour;
    uint8_t a2_day_date;

    // e
    uint8_t control;
    uint8_t status;
    uint8_t aging;
    uint16_t temp;
} __attribute__((packed));

GCT_CONTAINER_FCNS(dev_request_queue, static, dev_request_queue,
                   remove, push);

struct ds3231_priv_s
{
    struct dev_i2c_rq_s i2c_req;
    struct dev_i2c_transfer_s i2c_transfer[2];
    struct ds3231_regs regs;
    struct device_i2c_s i2c;
    uint8_t saddr;
    uint8_t reg;
    struct dev_valio_rq_s *pending;

    dev_request_queue_root_t queue;
};

static void ds3231_request_run(
    struct device_s *dev,
    struct ds3231_priv_s *pv);

static inline uint8_t bcd2dec(uint8_t x)
{
    return ((x >> 4) * 10) | (x & 0xf);
}

static inline uint8_t dec2bcd(uint8_t x)
{
    return ((x / 10) << 4) | (x % 10);
}

static inline uint16_t year2dec(uint8_t x)
{
    return 2000 + bcd2dec(x);
}

static inline uint8_t dec2year(uint16_t x)
{
    return dec2bcd(x - 2000);
}

static KROUTINE_EXEC(ds3231_state_done)
{
    struct ds3231_priv_s *pv;
    struct dev_valio_rq_s *rq = NULL;
    struct device_s *dev;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.base.pvdata;

    LOCK_SPIN_IRQ(&dev->lock);

    rq = pv->pending;
    pv->pending = NULL;

    dev_request_queue_remove(&pv->queue, &rq->base);

    assert(rq);

    if (rq->type == DEVICE_VALIO_READ) {
        struct valio_hwclock_s *clk = rq->data;
    
        clk->sec = bcd2dec(pv->regs.sec);
        clk->min = bcd2dec(pv->regs.min);
        clk->hour = bcd2dec(pv->regs.hour);
        clk->day = bcd2dec(pv->regs.day);
        clk->date = bcd2dec(pv->regs.date);
        clk->month = bcd2dec(pv->regs.month);
        clk->year = year2dec(pv->regs.year);
    }

    kroutine_exec(&rq->base.kr);

    ds3231_request_run(dev, pv);
    LOCK_RELEASE_IRQ(&dev->lock);
}

static void ds3231_request_run(
    struct device_s *dev,
    struct ds3231_priv_s *pv)
{
    bool_t empty = dev_request_queue_isempty(&pv->queue);

    if (empty || pv->pending)
        return;

    struct dev_request_s *drq = dev_request_queue_head(&pv->queue);

    assert(drq);

    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(drq);

    kroutine_init_immediate(&pv->i2c_req.base.kr, ds3231_state_done);

    pv->i2c_req.base.pvdata = dev;
    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_req.saddr = pv->saddr;
    pv->reg = 0;

    pv->i2c_transfer[0].data = &pv->reg;
    pv->i2c_transfer[0].size = 1;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_transfer[1].data = (uint8_t *)&pv->regs;
    pv->i2c_transfer[1].size = 8;
    pv->i2c_req.transfer_count = 2;

    if (rq->type == DEVICE_VALIO_READ) {
        pv->i2c_req.transfer[1].type = DEV_I2C_READ;
    } else {
        const struct valio_hwclock_s *clk = rq->data;

        pv->i2c_req.transfer[1].type = DEV_I2C_WRITE;

        pv->regs.sec = dec2bcd(clk->sec);
        pv->regs.min = dec2bcd(clk->min);
        pv->regs.hour = dec2bcd(clk->hour);
        pv->regs.day = dec2bcd(clk->day);
        pv->regs.date = dec2bcd(clk->date);
        pv->regs.month = dec2bcd(clk->month);
        pv->regs.year = dec2year(clk->year);
    }

    pv->pending = rq;

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static DEV_VALIO_REQUEST(ds3231_request)
{
    struct device_s *dev = accessor->dev;
    struct ds3231_priv_s *pv = dev->drv_pv;
    error_t err = -EINVAL;

    dprintk("%s\n", __FUNCTION__);

    LOCK_SPIN_IRQ(&dev->lock);

    if (req->attribute == VALIO_HWCLOCK
        && (req->type == DEVICE_VALIO_READ
            || req->type == DEVICE_VALIO_WRITE)) {
        err = 1;
        dev_request_queue_push(&pv->queue, &req->base);
        ds3231_request_run(dev, pv);
    }

    LOCK_RELEASE_IRQ(&dev->lock);

    if (err <= 0) {
        req->error = err;
        
        kroutine_exec(&req->base.kr);
    }
}

static DEV_INIT(ds3231_init);
static DEV_CLEANUP(ds3231_cleanup);

#define ds3231_use dev_use_generic
#define ds3231_cancel (dev_valio_cancel_t*)&dev_driver_notsup_fcn

DRIVER_DECLARE(ds3231_drv, 0, "DS3231 Calendar Clock", ds3231,
               DRIVER_VALIO_METHODS(ds3231));

DRIVER_REGISTER(ds3231_drv);

static DEV_INIT(ds3231_init)
{
    struct ds3231_priv_s *pv;


    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    if (!pv)
        return -ENOMEM;

    memset(pv, 0, sizeof(*pv));

    if (device_get_param_dev_accessor(
            dev, "bus", &pv->i2c, DRIVER_CLASS_I2C)) {
        printk("Bus not found\n");
        goto err_pv;
    }

    if (dev_i2c_res_get_addr(dev, &pv->saddr, 0)) {
        printk("Address not found\n");
        goto err_pv;
    }

    dev_request_queue_init(&pv->queue);

    dev->drv_pv = pv;

    return 0;

  err_pv:
    mem_free(pv);

    return -1;
}

static DEV_CLEANUP(ds3231_cleanup)
{
    struct ds3231_priv_s *pv = dev->drv_pv;

    device_put_accessor(&pv->i2c.base);
    dev_request_queue_destroy(&pv->queue);
    mem_free(pv);
}
