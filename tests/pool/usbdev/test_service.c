/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014

*/

#include "test_service_desc.h"
#include <mutek/printk.h>

struct usbdev_test_info_s
{
  struct dev_usbdev_rq_s tr;
  const struct usb_endpoint_descriptor_s * edesc;
  uint32_t last;
  uint16_t base;
  uint16_t max;
  uint8_t * buffer;
  char * desc;
};

STRUCT_INHERIT(usbdev_test_info_s, dev_usbdev_rq_s, tr);

struct usbdev_test_service_s
{
  struct usbdev_service_s service;
  /* Associated USB controller device */
  struct device_usbdev_s usb;

  /* Sequence for kroutine */
  struct kroutine_sequence_s seq;

  bool_t service_enabled;
  /* Usb stack request endpoint 0 */
  struct usbdev_service_rq_s rq;

#if USBDEV_SERV_TEST_CONTROL_0_ENDPOINT
  uint16_t cnt0, base0;
  uint32_t last0;
#endif
#if USBDEV_SERV_TEST_CONTROL_N_ENDPOINT
  struct usbdev_test_info_s ctrl;
  uint16_t cntn;
  struct usb_ctrl_setup_s setup;
#endif
#if USBDEV_SERV_TEST_BULK_ENDPOINT
  /* write/read bulk endpoints */
  struct usbdev_test_info_s wbulk;
  struct usbdev_test_info_s rbulk;
  bool_t alt;
#endif
#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
  /* 2 transfers on each endpoints */
  struct usbdev_test_info_s wiso[2];
  struct usbdev_test_info_s riso[2];
#endif
#if USBDEV_SERV_TEST_INTERRUPT_ENDPOINT
  struct usbdev_test_info_s wirq;
  struct usbdev_test_info_s rirq;
#endif
};

struct usbdev_test_service_s pv;

static void lfsr_data(uint8_t *data, size_t len, uint32_t *sd)
{
  uint32_t seed = *sd;

  if (!seed)
    seed++;
  while (len--)
    {
      seed = (~((seed & 1) - 1) & 0x8a523d7c) ^ (seed >> 1);
      *data++ = seed ^ (seed >> 8) ^ (seed >> 16) ^ (seed >> 24);
    }

  *sd = seed;
}

static bool_t lfsr_data_check(uint8_t *data, size_t len, uint32_t *sd)
{
  uint32_t seed = *sd;

  if (!seed)
    seed++;

  for (uint32_t i = 0; i < len; i++)
    {
      seed = (~((seed & 1) - 1) & 0x8a523d7c) ^ (seed >> 1);
      uint8_t ref = seed ^ (seed >> 8) ^ (seed >> 16) ^ (seed >> 24);

      if (data[i] != ref)
        {
          printk("Read error data[%d] = 0x%x ref = 0x%x\n", i, data[i], ref);
          return 1;
        }
    }

  *sd = seed;
  return 0;
}

static void usbdev_service_start_transfer(struct usbdev_test_service_s *pv,
                                          struct usbdev_test_info_s * info)
{
  struct dev_usbdev_rq_s *tr = &info->tr;
  /* Push transfer to stack */
  error_t err = usbdev_stack_transfer(&pv->usb, &pv->service, tr, info->edesc);

  while(1)
  {
    switch (err)
      {
        default:
          printk("Error %d on posting request on %s\n", err, info->desc);
        case 0:
          return;
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
        case -EAGAIN:
          assert(info == &pv->wbulk);
          /* Change endpoint of transfer */
          info->edesc = tr->rev ? &ep_in_1 : &ep_in_0;
          /* Push transfer to stack */
          err = usbdev_stack_transfer(&pv->usb, &pv->service, tr, info->edesc);
          break;
#endif
      }
  }
}

static void usbdev_service_test_write(struct usbdev_test_service_s *pv,
                                      struct usbdev_test_info_s * info)
{
  if(info->base % 2)
    info->base = info->max;
  else
    info->base = rand() % info->max;

  if (!info->base)
    info->base++;

  lfsr_data(info->buffer, info->base, &info->last); 

  struct dev_usbdev_rq_s *tr = &info->tr;

  tr->size = info->base;
  tr->data = info->buffer;
  tr->error = 0;

#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
  if (info == &pv->wiso[0] || info == &pv->wiso[1])
    {
      pv->wiso[0].last = info->last;
      pv->wiso[1].last = info->last;
    }
#endif
  usbdev_service_start_transfer(pv, info);
}

static void usbdev_service_test_read(struct usbdev_test_service_s *pv,
                                     struct usbdev_test_info_s * info)
{
  struct dev_usbdev_rq_s *tr = &info->tr;

  info->base = info->max;

  tr->data = info->buffer;
  tr->size = info->max;
  tr->error = 0;

  usbdev_service_start_transfer(pv, info);
}

static KROUTINE_EXEC(usbdev_service_test_write_cb)
{
  struct dev_usbdev_rq_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct usbdev_test_info_s * info = usbdev_test_info_s_cast(tr);
  struct usbdev_test_service_s *pv = tr->base.pvdata;


  switch(tr->error)
    {
    case -EPIPE:
      /* USB reset or disconnected */
      printk("-EPIPE on %s\n", info->desc);
      return;
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
    case -EAGAIN:
      {
         uint16_t done = info->base - tr->size;
         usbdev_test_printk("-EAGAIN on %s\n", info->desc);
         /* Interface reconfiguration */
         if (tr->size == 0)
         /* All data have been sent */
           break;
         info->base = tr->size;
         tr->data = info->buffer;
         if (done)
           memcpy(info->buffer, info->buffer + done, tr->size);
         /* Change endpoint of transfer */
         info->edesc = tr->rev ? &ep_in_1 : &ep_in_0;
         /* Push transfer to stack */
         usbdev_service_start_transfer(pv, info);
         return;
      }
#endif
    default:
      break;
    }

  usbdev_test_printk("Write %d on %s\n", info->base - tr->size, info->desc);

#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
  if (info == &pv->wiso[0] || info == &pv->wiso[1])
    {
      if (tr->error == -ETIMEDOUT)
        printk("drop packet on write\n");
    }
#endif

  /* Check if service is enabled */
  if (pv->service_enabled)
    usbdev_service_test_write(pv, info);
}

static KROUTINE_EXEC(usbdev_service_test_read_cb)
{
  struct dev_usbdev_rq_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct usbdev_test_info_s * info = usbdev_test_info_s_cast(tr);
  struct usbdev_test_service_s *pv = tr->base.pvdata;

  switch(tr->error)
    {
    case -EPIPE:
      /* USB reset or disconnected */
      printk("-EPIPE on %s\n", info->desc);
      return;
    default:
      break;
    }

#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
  if (info == &pv->riso[0] || info == &pv->riso[1])
  /* Isochronous transfert */
    {
      if (tr->error == -ETIMEDOUT)
        printk("drop packet on read\n");
    }
#endif

  uint32_t done = info->base - tr->size;

  usbdev_test_printk("Read %d on %s\n", done, info->desc);

  if (done && lfsr_data_check(info->buffer, done, &info->last))
    {
      printk("Error on %s\n", info->desc);
      assert(0);
    }

#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
  if (info == &pv->riso[0] || info == &pv->riso[1])
  /* Isochronous transfert */
    {
      pv->riso[0].last = info->last;
      pv->riso[1].last = info->last;
    }
#endif

  if (pv->service_enabled)
    usbdev_service_test_read(pv, info);
}

static KROUTINE_EXEC(usbdev_test_ctrl_cb);

static void usbdev_test_service_disconnect(struct usbdev_test_service_s *pv)
{
  usbdev_test_printk("USBDEV test service disabled\n");
  pv->service_enabled = 0;

  pv->rq.type = USBDEV_GET_COMMAND;
  dev_timer_rq_init_seq(pv, &usbdev_test_ctrl_cb, &pv->seq);
  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

#if USBDEV_SERV_TEST_CONTROL_N_ENDPOINT
static KROUTINE_EXEC(usbdev_test_ctrl_n_cb)
{
  struct dev_usbdev_rq_s *tr = KROUTINE_CONTAINER(kr, *tr, base.kr);
  struct usbdev_test_info_s * info = usbdev_test_info_s_cast(tr);
  struct usbdev_test_service_s *pv = tr->base.pvdata;
  uint16_t done;

  if (tr->error == -EPIPE)
  /* USB reset or disconnected */
    {
      printk("cb -EPIPE on %s\n", info->desc);
      return;
    }

  uint16_t len = usb_setup_length_get(&pv->setup);

  switch(tr->type)
    {
    case DEV_USBDEV_CTRL_SETUP:
        tr->data = pv->ctrl.buffer;
        tr->rem = len;
        pv->cntn = 0;
        if (len == 0)
          {
            tr->type = DEV_USBDEV_CTRL_STATUS_IN;
            tr->size = 0;
          }
        else if (usb_setup_direction_get(&pv->setup) == USB_DEVICE_TO_HOST)
        /* Send data */ 
          {
            tr->type = DEV_USBDEV_DATA_IN;
            tr->size = len > USB_TEST_CONTROL_SIZE ? USB_TEST_CONTROL_SIZE : len;
            /* Copy data in provided buffer */
            lfsr_data(pv->ctrl.buffer, tr->size, &pv->ctrl.last); 
          }
        else
        /* Receive data */ 
          {
            tr->type = DEV_USBDEV_DATA_OUT;
            tr->size = USB_TEST_CONTROL_SIZE;
          }
        pv->ctrl.base = tr->size;
        break;

    case DEV_USBDEV_DATA_IN:

      done = pv->ctrl.base - tr->size;

      usbdev_test_printk("Write %d on %s\n", done, info->desc);

      pv->cntn += done;

      tr->rem = len - pv->cntn;
      tr->data = pv->ctrl.buffer;

      if (tr->rem > USB_TEST_CONTROL_SIZE)
        tr->size = USB_TEST_CONTROL_SIZE;
      else
        tr->size = tr->rem;

      if (pv->cntn < len)
        {
          pv->ctrl.base = tr->size;
          /* Copy data in provided buffer */
          lfsr_data(pv->ctrl.buffer, tr->size, &pv->ctrl.last); 
        }
      else
      /* End of data stage */
        {
          tr->type = DEV_USBDEV_CTRL_STATUS_OUT;
          tr->size = 0;
        }
      break;

    case DEV_USBDEV_DATA_OUT:
  
      done = pv->ctrl.base - tr->size;
  
      usbdev_test_printk("Read %d on %s\n", done, info->desc);
  
      if (lfsr_data_check(pv->ctrl.buffer, done, &pv->ctrl.last))
        {
          printk("Error on endpoint N read\n");
          assert(0);
        }
  
      pv->cntn += done;

      if (pv->cntn < len)
        {
          tr->data = pv->ctrl.buffer;
          tr->size = USB_TEST_CONTROL_SIZE;
          tr->rem = len - pv->cntn;
          pv->ctrl.base = tr->size;
        }
      else
      /* End of data stage */
        {
          tr->type = DEV_USBDEV_CTRL_STATUS_IN;
          tr->size = USB_TEST_CONTROL_SIZE;
        }
      break;
    case DEV_USBDEV_CTRL_STATUS_IN:
    case DEV_USBDEV_CTRL_STATUS_OUT:
      tr->type = DEV_USBDEV_CTRL_SETUP;
      tr->size = 8;
      tr->data = (void*)&pv->setup;
      break;

    default:
      break;
  }
  usbdev_service_start_transfer(pv, info);
}
#endif

#if USBDEV_SERV_TEST_CONTROL_0_ENDPOINT
static KROUTINE_EXEC(usbdev_test_read_ctrl_0_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct usbdev_test_service_s *pv = rq->pvdata;

  switch(rq->error)
    {
    case -EPIPE:
      usbdev_test_printk("USBDEV -EPIPE on 0\n");
      return usbdev_test_service_disconnect(pv);
    case -EIO:
      usbdev_test_printk("USBDEV -EIO on 0\n");
      abort();
    default:
      break;
    }

  uint16_t size = rq->ctrl.size;

  usbdev_test_printk("Read %d on 0\n", size);

  if (lfsr_data_check(rq->ctrl.buffer, size, &pv->last0))
    {
      printk("Error on endpoint 0 read\n");
      ensure(0);
    }

  pv->cnt0 += size;

  uint16_t len = usb_setup_length_get(rq->ctrl.setup);

  rq->error = 0;

  if (pv->cnt0 >= len)
  /* End of data stage */
    {
//      printk("%d\n", pv->cnt0);
      rq->type = USBDEV_GET_COMMAND;
      kroutine_init_deferred_seq(&rq->kr, &usbdev_test_ctrl_cb, &pv->seq);
    }

  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}

static KROUTINE_EXEC(usbdev_test_write_ctrl_0_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct usbdev_test_service_s *pv = rq->pvdata;

  switch(rq->error)
    {
    case -EPIPE:
      usbdev_test_printk("USBDEV -EPIPE on 0\n");
      return usbdev_test_service_disconnect(pv);
    case -EIO:
      usbdev_test_printk("USBDEV -EIO on 0\n");
      abort();
    default:
      break;
    }

  usbdev_test_printk("Write %d on 0\n", pv->base0);

  pv->cnt0 += pv->base0;
     
  uint16_t len = usb_setup_length_get(rq->ctrl.setup);
  uint16_t size = len - pv->cnt0;

  if (size)
  /* Introduce prematurate end of transfer */
    {
      if (!(rand() % 256))
        {
          printk("ZLP %d\n", size);
          rq->type = USBDEV_GET_COMMAND;
          kroutine_init_deferred_seq(&rq->kr, &usbdev_test_ctrl_cb, &pv->seq);
        }
      else
        {
          if (size > rq->ctrl.size)
            size = rq->ctrl.size;
         
          pv->base0 = size;
          rq->error = 0;
          rq->ctrl.size = size;
          rq->type = USBDEV_TRANSFER_DATA;
         
          /* Copy data in provided buffer */
          lfsr_data(rq->ctrl.buffer, size, &pv->last0); 
        }
    }
  else
  /* End of data stage */
    {
      rq->type = USBDEV_GET_COMMAND;
      kroutine_init_deferred_seq(&rq->kr, &usbdev_test_ctrl_cb, &pv->seq);
    }

  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}
#endif

static KROUTINE_EXEC(usbdev_test_ctrl_cb)
{
  struct usbdev_service_rq_s *rq =  KROUTINE_CONTAINER(kr, *rq, kr);
  struct usbdev_test_service_s *pv = rq->pvdata;

  switch (rq->cmd)
    {
    case USBDEV_ENABLE_SERVICE:
      usbdev_test_printk("USBDEV test service enabled\n");
      /* Enable service */
      pv->service_enabled = 1;
#if USBDEV_SERV_TEST_CONTROL_0_ENDPOINT
      pv->cnt0 = 0;
      pv->last0 = 0;
#endif
#if USBDEV_SERV_TEST_CONTROL_N_ENDPOINT
      pv->cntn = 0;
      pv->ctrl.last = 0;

      pv->ctrl.tr.type = DEV_USBDEV_CTRL_SETUP;
      pv->ctrl.tr.size = 8;
      pv->ctrl.tr.data = (uint8_t *)&pv->setup;

      usbdev_service_start_transfer(pv, &pv->ctrl);
#endif
#if USBDEV_SERV_TEST_BULK_ENDPOINT
      pv->wbulk.last = 0;
      pv->wbulk.base = 0;
      pv->rbulk.last = 0;
      pv->rbulk.base = 0;

      usbdev_service_test_read(pv, &pv->rbulk);
      usbdev_service_test_write(pv, &pv->wbulk);
#endif
#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
      for (uint8_t i = 0; i< 2; i++)
        {
          pv->wiso[i].last = 0;
          pv->wiso[i].base = 0;
          pv->riso[i].last = 0;
          pv->riso[i].base = 0;
        }

      for (uint8_t i=0; i<2; i++)
        usbdev_service_test_read(pv, &pv->riso[i]);
      for (uint8_t i=0; i<2; i++)
        usbdev_service_test_write(pv, &pv->wiso[i]);
#endif
#if USBDEV_SERV_TEST_INTERRUPT_ENDPOINT
      pv->rirq.last = 0;
      pv->rirq.base = 0;
      pv->wirq.last = 0;
      pv->wirq.base = 0;

      usbdev_service_test_read(pv, &pv->rirq);
      usbdev_service_test_write(pv, &pv->wirq);
#endif
      break;
    
    case USBDEV_DISABLE_SERVICE:
      return usbdev_test_service_disconnect(pv);
    
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
    case USBDEV_CHANGE_INTERFACE:
      usbdev_test_printk("USBDEV change interface\n");
      break;
#endif

#if USBDEV_SERV_TEST_CONTROL_0_ENDPOINT
    case USBDEV_PROCESS_CONTROL:
      {
        uint16_t len = usb_setup_length_get(rq->ctrl.setup);
        size_t size = len;
     
        pv->cnt0 = 0;
     
        if (len == 0)
          break;

        /* Data stage */
        rq->type = USBDEV_TRANSFER_DATA;
     
        if (usb_setup_direction_get(rq->ctrl.setup) == USB_DEVICE_TO_HOST)
        /* We must send data */ 
          {
            if (len > rq->ctrl.size)
              size = rq->ctrl.size;
     
            rq->ctrl.size = size;

            /* Copy data in provided buffer */
            lfsr_data(rq->ctrl.buffer, size, &pv->last0); 

            pv->base0 = size;
            kroutine_init_deferred_seq(&rq->kr, &usbdev_test_write_ctrl_0_cb, &pv->seq);
          }
        else
        /* We must retrieve data */ 
          kroutine_init_deferred_seq(&rq->kr, &usbdev_test_read_ctrl_0_cb, &pv->seq);
        
        }
      break;
#endif

    default:
      break;
  }

  rq->error = 0;
  /* Push request on stack */
  usbdev_stack_request(&pv->usb, &pv->service, &pv->rq);
}
  
void app_start()
{
  printk("START\n");

  memset(&pv, 0, sizeof(struct usbdev_test_service_s));

  error_t err = device_get_accessor_by_path(&pv.usb.base, NULL, USBDEV_TEST_DEVICE_PATH,
                                            DRIVER_CLASS_USBDEV);

  if (err)
    {
      printk("Error on getting USBDEV accessor %d\n", err);
      return;
    }

#if defined(CONFIG_DRIVER_USB_SYNOPSYS_EFM32) 
  struct device_cmu_s clock;
  ensure(!device_get_accessor_by_path(&clock.base, NULL, "recmu*", DRIVER_CLASS_CMU));
  /* Switch to clock configuration 3 */
  ensure(!(DEVICE_OP(&clock, app_configid_set, 3)));
#endif

  struct dev_usbdev_rq_s *tr;

#if USBDEV_SERV_TEST_BULK_ENDPOINT
  pv.rbulk.buffer = usbdev_stack_allocate(&pv.usb, USBDEV_TEST_BULK_BUFFER_SIZE);

  assert(pv.rbulk.buffer);

  pv.rbulk.max = USBDEV_TEST_BULK_BUFFER_SIZE;
  pv.rbulk.edesc = &ep_out_0;
  pv.rbulk.desc = "read endpoint of Bulk interface";

  tr = &pv.rbulk.tr;
  tr->type = DEV_USBDEV_DATA_OUT;
  tr->base.pvdata = &pv;
  dev_usbdev_rq_init_seq(tr, &usbdev_service_test_read_cb, &pv.seq);

  pv.wbulk.buffer = usbdev_stack_allocate(&pv.usb, USBDEV_TEST_BULK_BUFFER_SIZE);

  assert(pv.wbulk.buffer);

  pv.wbulk.max = USBDEV_TEST_BULK_BUFFER_SIZE;
  pv.wbulk.desc = "write endpoint of Bulk interface";
  pv.wbulk.edesc = &ep_in_0;

  tr = &pv.wbulk.tr;
  tr->type = DEV_USBDEV_DATA_IN;
  tr->base.pvdata = &pv;
  dev_usbdev_rq_init_seq(tr, &usbdev_service_test_write_cb, &pv.seq);

#endif
#if USBDEV_SERV_TEST_ISOCHRONOUS_ENDPOINT
  for (uint8_t i=0; i<2; i++)
    {
      pv.riso[i].buffer = usbdev_stack_allocate(&pv.usb, USB_TEST_ISOCHONOUS_SIZE);
      pv.riso[i].max = USB_TEST_ISOCHONOUS_SIZE;
      pv.riso[i].edesc = &ep_out_2;
      pv.riso[i].desc = "read endpoint of Isochronous interface";

      tr = &pv.riso[i].tr;
      tr->type = DEV_USBDEV_PARTIAL_DATA_OUT;
      tr->base.pvdata = &pv;
      dev_usbdev_rq_init_seq(tr, &usbdev_service_test_read_cb, &pv.seq);
      
      pv.wiso[i].buffer = usbdev_stack_allocate(&pv.usb, USB_TEST_ISOCHONOUS_SIZE);
      pv.wiso[i].max = USB_TEST_ISOCHONOUS_SIZE;
      pv.wiso[i].edesc = &ep_in_2;
      pv.wiso[i].desc = "write endpoint of Isochronous interface";

      tr = &pv.wiso[i].tr;
      tr->type = DEV_USBDEV_DATA_IN;
      tr->base.pvdata = &pv;
      dev_usbdev_rq_init_seq(tr, &usbdev_service_test_write_cb, &pv.seq);
    }
#endif
#if USBDEV_SERV_TEST_INTERRUPT_ENDPOINT
   pv.wirq.buffer = usbdev_stack_allocate(&pv.usb, USB_TEST_INTERRUPT_SIZE);
   pv.wirq.max = USB_TEST_INTERRUPT_SIZE;
   pv.wirq.edesc = &ep_in_3;
   pv.wirq.desc = "write endpoint of Interrupt interface";

   tr = &pv.rirq.tr;
   tr->type = DEV_USBDEV_PARTIAL_DATA_OUT;
   tr->base.pvdata = &pv;
   dev_usbdev_rq_init_seq(tr, &usbdev_service_test_read_cb, &pv.seq);
   
   pv.rirq.buffer = usbdev_stack_allocate(&pv.usb, USB_TEST_INTERRUPT_SIZE);
   pv.rirq.max = USB_TEST_INTERRUPT_SIZE;
   pv.rirq.edesc = &ep_out_3;
   pv.rirq.desc = "read endpoint of Interrupt interface";

   tr = &pv.wirq.tr;
   tr->type = DEV_USBDEV_DATA_IN;
   tr->base.pvdata = &pv;
   dev_usbdev_rq_init_seq(tr, &usbdev_service_test_write_cb, &pv.seq);
#endif
#if USBDEV_SERV_TEST_CONTROL_N_ENDPOINT
  pv.ctrl.buffer = usbdev_stack_allocate(&pv.usb, USB_TEST_CONTROL_SIZE);
  pv.ctrl.max = USB_TEST_CONTROL_SIZE;
  pv.ctrl.edesc = &ep_4;
  pv.ctrl.desc = "endpoint of additionnal control interface";

  tr = &pv.ctrl.tr;
  tr->base.pvdata = &pv;
  dev_usbdev_rq_init_seq(tr, &usbdev_test_ctrl_n_cb, &pv.seq);
#endif

  /* Attach description to USB device */
  usbdev_stack_set_device_info(&pv.usb, &usbdevinfo);
  pv.service.desc = &usb_test_service;

  /* Endpoint map */
  pv.service.start.epo[0] = 0x5321;
  pv.service.start.epi[0] = 0x5432;
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  pv.service.start.epi[1] = 0x0001;
  pv.service.start.epo[1] = 0x0001;
#endif

  ensure(usbdev_stack_service_register(&pv.usb, &pv.service) == 0);

  struct usbdev_service_rq_s *rq = &pv.rq;

  rq->pvdata = &pv;
  rq->type = USBDEV_GET_COMMAND; 
  rq->error = 0;

  kroutine_seq_init(&pv.seq);
  kroutine_init_deferred_seq(&rq->kr, &usbdev_test_ctrl_cb, &pv.seq);

  /* Push initial request on stack */
  usbdev_stack_request(&pv.usb, &pv.service, &pv.rq);

  /* Start USB device */
  device_start(&pv.usb.base);

}

