
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/driver.h>
#include <device/request.h>
#include <device/class/valio.h>
#include <device/valio/motion.h>

#include <device/class/i2c.h>


/* Registers of ADXL345 */
#define ADXL345_REG_DEVID           0x00
#define ADXL345_REG_THRESH_TAP      0x1d
#define ADXL345_REG_OFSX            0x1e
#define ADXL345_REG_OFSY            0x1f
#define ADXL345_REG_OFSZ            0x20
#define ADXL345_REG_DUR             0x21
#define ADXL345_REG_LATENT          0x22
#define ADXL345_REG_WINDOW          0x23
#define ADXL345_REG_THRESH_ACT      0x24
#define ADXL345_REG_THRESH_INACT    0x25
#define ADXL345_REG_TIME_INACT      0x26
#define ADXL345_REG_ACT_INACT_CTL   0x27
#define ADXL345_REG_THRESH_FF       0x28
#define ADXL345_REG_TIME_FF         0x29
#define ADXL345_REG_TAP_AXES        0x2a
#define ADXL345_REG_ACT_TOP_STATUS  0x2b
#define ADXL345_REG_BW_RATE         0x2c
#define ADXL345_REG_POWER_CTL       0x2d
#define ADXL345_REG_INT_ENABLE      0x2e
#define ADXL345_REG_INT_MAP         0x2f
#define ADXL345_REG_INT_SOURCE      0x30
#define ADXL345_REG_DATA_FORMAT     0x31
#define ADXL345_REG_DATAX0          0x32
#define ADXL345_REG_DATAX1          0x33
#define ADXL345_REG_DATAY0          0x34
#define ADXL345_REG_DATAY1          0x35
#define ADXL345_REG_DATAZ0          0x36
#define ADXL345_REG_DATAZ1          0x37
#define ADXL345_REG_FIFO_CTL        0x38
#define ADXL345_REG_FIFO_STATUS     0x39

GCT_CONTAINER_FCNS(dev_request_queue, static, dev_request_queue,
                   push);

struct adxl345_private_s
{
    struct device_i2c_s         i2c;
    uint8_t                     i2c_saddr;
    struct dev_i2c_rq_s         i2c_req;
    struct dev_i2c_transfer_s   i2c_transfer[2];

    uint8_t                     wdata[6];
    uint8_t                     rdata[6];

    dev_request_queue_root_t    queue;
};

static
void adxl345_request_run(struct device_s *dev, struct adxl345_private_s *pv);

static
KROUTINE_EXEC(adxl345_offset_write_done)
{
    struct device_s          *dev;
    struct adxl345_private_s *pv;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.base.pvdata;

    struct dev_request_s  *base = dev_request_queue_head(&pv->queue);
    struct dev_valio_rq_s *rq   = dev_valio_rq_s_from_base(base);

    LOCK_SPIN_IRQ(&dev->lock);

    rq->error = 0;
    dev_request_queue_pop(&pv->queue);

    LOCK_RELEASE_IRQ(&dev->lock);
    kroutine_exec(&rq->base.kr, cpu_is_interruptible());
    LOCK_SPIN_IRQ(&dev->lock);

    adxl345_request_run(dev, pv);
    LOCK_RELEASE_IRQ(&dev->lock);
}

static
void adxl345_do_offset_write(struct device_s          *dev,
                             struct dev_valio_rq_s    *rq,
                             struct adxl345_private_s *pv)
{
    struct valio_motion_axis_data_s *val = rq->data;

    pv->i2c_req.transfer    = pv->i2c_transfer;
    pv->i2c_req.saddr       = pv->i2c_saddr;

    pv->i2c_transfer[0].data = pv->wdata;
    pv->i2c_transfer[0].size = 4;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->wdata[0] = ADXL345_REG_OFSX;
    pv->wdata[1] = (uint8_t) (val->x & 0xff);
    pv->wdata[2] = (uint8_t) (val->y & 0xff);
    pv->wdata[3] = (uint8_t) (val->z & 0xff);

    pv->i2c_req.transfer_count = 1;

    pv->i2c_req.base.pvdata = dev;
    kroutine_init(&pv->i2c_req.base.kr, &adxl345_offset_write_done,
                  KROUTINE_IMMEDIATE);

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static
KROUTINE_EXEC(adxl345_offset_read_done)
{
    struct device_s          *dev;
    struct adxl345_private_s *pv;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.base.pvdata;

    struct dev_request_s            *base = dev_request_queue_head(&pv->queue);
    struct dev_valio_rq_s           *rq   = dev_valio_rq_s_from_base(base);
    struct valio_motion_axis_data_s *val  = rq->data;

    LOCK_SPIN_IRQ(&dev->lock);

    val->x = (int32_t) pv->rdata[0];
    val->y = (int32_t) pv->rdata[1];
    val->z = (int32_t) pv->rdata[2];

    rq->error = 0;
    dev_request_queue_pop(&pv->queue);

    LOCK_RELEASE_IRQ(&dev->lock);
    kroutine_exec(&rq->base.kr, cpu_is_interruptible());
    LOCK_SPIN_IRQ(&dev->lock);

    adxl345_request_run(dev, pv);
    LOCK_RELEASE_IRQ(&dev->lock);
}

static
void adxl345_do_offset_read(struct device_s          *dev,
                            struct dev_valio_rq_s    *rq,
                            struct adxl345_private_s *pv)
{
    pv->i2c_req.transfer    = pv->i2c_transfer;
    pv->i2c_req.saddr       = pv->i2c_saddr;

    pv->i2c_transfer[0].data = pv->wdata;
    pv->i2c_transfer[0].size = 1;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->wdata[0] = ADXL345_REG_OFSX;

    pv->i2c_transfer[1].data = pv->rdata;
    pv->i2c_transfer[1].size = 3;
    pv->i2c_transfer[1].type = DEV_I2C_READ;

    pv->i2c_req.transfer_count = 2;

    pv->i2c_req.base.pvdata = dev;
    kroutine_init(&pv->i2c_req.base.kr, &adxl345_offset_read_done,
                  KROUTINE_IMMEDIATE);

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static
KROUTINE_EXEC(adxl345_data_read_done)
{
    struct device_s          *dev;
    struct adxl345_private_s *pv;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.base.pvdata;

    struct dev_request_s       *base = dev_request_queue_head(&pv->queue);
    struct dev_valio_rq_s      *rq   = dev_valio_rq_s_from_base(base);
    struct valio_motion_data_s *data  = rq->data;

    LOCK_SPIN_IRQ(&dev->lock);

    data->accel.axis = VALIO_MOTION_ACC_XYZ;
    data->accel.x = ((int16_t) pv->rdata[1] << 8) | pv->rdata[0];
    data->accel.y = ((int16_t) pv->rdata[3] << 8) | pv->rdata[2];
    data->accel.z = ((int16_t) pv->rdata[5] << 8) | pv->rdata[4];

    data->gyro.axis = 0;
    data->comp.axis = 0;

    dev_request_queue_pop(&pv->queue);

    rq->error = 0;

    LOCK_RELEASE_IRQ(&dev->lock);
    kroutine_exec(&rq->base.kr, cpu_is_interruptible());
    LOCK_SPIN_IRQ(&dev->lock);

    adxl345_request_run(dev, pv);
    LOCK_RELEASE_IRQ(&dev->lock);
}

static
void adxl345_do_data_read(struct device_s          *dev,
                           struct dev_valio_rq_s    *rq,
                           struct adxl345_private_s *pv)
{
    pv->i2c_req.transfer    = pv->i2c_transfer;
    pv->i2c_req.saddr       = pv->i2c_saddr;

    pv->i2c_transfer[0].data = pv->wdata;
    pv->i2c_transfer[0].size = 1;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->wdata[0] = ADXL345_REG_DATAX0;

    pv->i2c_transfer[1].data = pv->rdata;
    pv->i2c_transfer[1].size = 6;
    pv->i2c_transfer[1].type = DEV_I2C_READ;

    pv->i2c_req.transfer_count = 2;

    pv->i2c_req.base.pvdata = dev;
    kroutine_init(&pv->i2c_req.base.kr, &adxl345_data_read_done,
                  KROUTINE_IMMEDIATE);

    DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static
void adxl345_request_run(struct device_s *dev, struct adxl345_private_s *pv)
{
    struct dev_request_s  *base;
    struct dev_valio_rq_s *rq;

    if (dev_request_queue_isempty(&pv->queue))
        return;

    base = dev_request_queue_head(&pv->queue);
    assert(base != NULL);

    rq = dev_valio_rq_s_from_base(base);

    switch (rq->type)
    {
    default:
        assert(!"impossible reach!");
        break;

    case DEVICE_VALIO_READ:
        switch (rq->attribute)
        {
        default:
            assert(!"impossible reach");
            break;

        case VALIO_MOTION_ACCEL_OFST:
            adxl345_do_offset_read(dev, rq, pv);
            break;

        case VALIO_MOTION_DATA:
            adxl345_do_data_read(dev, rq, pv);
            break;
        }
        break;

    case DEVICE_VALIO_WRITE:
        assert(rq->attribute == VALIO_MOTION_ACCEL_OFST);
        adxl345_do_offset_write(dev, rq, pv);
        break;
    }
}

static
DEV_VALIO_REQUEST(adxl345_request)
{
    struct device_s          *dev = accessor->dev;
    struct adxl345_private_s *pv  = dev->drv_pv;

    error_t err = -EINVAL;

    LOCK_SPIN_IRQ(&dev->lock);

    switch (req->type)
    {
    default:
        break;

    case DEVICE_VALIO_READ:
        switch (req->attribute)
        {
        default:
            err = -ENOTSUP;
            break;

        case VALIO_MOTION_ACCEL_OFST:
        case VALIO_MOTION_DATA:
            dev_request_queue_push(&pv->queue, &req->base);
            err = 1;
            break;
        }
        break;

    case DEVICE_VALIO_WRITE:
        switch (req->attribute)
        {
        default:
            err = -ENOTSUP;
            break;

        case VALIO_MOTION_ACCEL_OFST:
            dev_request_queue_push(&pv->queue, &req->base);
            err = 1;
            break;
        }
        break;
    }

    if (err > 0)
        adxl345_request_run(dev, pv);

    LOCK_RELEASE_IRQ(&dev->lock);

    if (err < 0)
    {
        req->error = err;
        kroutine_exec(&req->base.kr, cpu_is_interruptible());
    }
}

static DEV_INIT(adxl345_init);
static DEV_CLEANUP(adxl345_cleanup);
static DEV_USE(adxl345_use);

DRIVER_DECLARE(adxl345_drv, "ADXL345 accelerometer", adxl345,
               DRIVER_VALIO_METHODS(adxl345));

DRIVER_REGISTER(adxl345_drv);

static
DEV_INIT(adxl345_init)
{
    struct adxl345_private_s *pv;

    dev->status = DEVICE_DRIVER_INIT_FAILED;

    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    memset(pv, 0, sizeof(*pv));
    if (!pv)
        return -ENOMEM;

    if (device_get_param_dev_accessor(dev, "bus", &pv->i2c, DRIVER_CLASS_I2C))
    {
        printk("Bus not found");
        goto err_pv;
    }

    if (dev_i2c_res_get_addr(dev, &pv->i2c_saddr, 0))
    {
        printk("Slave address not found");
        goto err_pv;
    }

    dev_request_queue_init(&pv->queue);

    dev->drv_pv = pv;
    dev->drv    = &adxl345_drv;
    dev->status = DEVICE_DRIVER_INIT_DONE;

    return 0;

err_pv:
    mem_free(pv);
    return 1;
}

static
DEV_CLEANUP(adxl345_cleanup)
{
    struct adxl345_private_s *pv = dev->drv_pv;

    device_put_accessor(&pv->i2c);
    dev_request_queue_destroy(&pv->queue);
    mem_free(pv);
}

static
error_t adxl345_start_stop(struct adxl345_private_s *pv, bool_t start)
{
    uint8_t data[2];

    data[0] = ADXL345_REG_POWER_CTL;
    data[1] = start ? 0x28 : 0x0;

#if defined(CONFIG_MUTEK_SCHEDULER)
    return dev_i2c_wait_write(&pv->i2c, pv->i2c_saddr, data, 2);
#else
    return dev_i2c_spin_write(&pv->i2c, pv->i2c_saddr, data, 2);
#endif
}

static
DEV_USE(adxl345_use)
{
    struct device_s          *dev = accessor->dev;
    struct adxl345_private_s *pv  = dev->drv_pv;

    switch (op)
    {
    default:
        break;

    case DEV_USE_START:
        return adxl345_start_stop(pv, 1);

    case DEV_USE_STOP:
        return adxl345_start_stop(pv, 0);
    }

    return 0;
}

