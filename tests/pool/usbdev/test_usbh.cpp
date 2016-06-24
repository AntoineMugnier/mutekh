#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>

#define USBTEST_TRANSFER_SIZE 512

//#define USBTEST_CTRL_0_ENABLE

#define USBTEST_INTERFACE_BULK_ENABLE
#define USBTEST_INTERFACE_BULK_ALTERNATE
#define USBTEST_INTERFACE_BULK 0
#define USBTEST_EP_BULK_IN 0x82
#define USBTEST_EP_BULK_OUT 0x1
#define USBTEST_EP_ALT_BULK_IN 0x82
#define USBTEST_EP_ALT_BULK_OUT 0x1

//#define USBTEST_INTERFACE_ISO_ENABLE
#define USBTEST_INTERFACE_ISO 1
#define USBTEST_EP_ISO_IN 0x83
#define USBTEST_EP_ISO_OUT 0x2

#define USBTEST_INTERFACE_IRQ_ENABLE
#define USBTEST_INTERFACE_IRQ 2
#define USBTEST_EP_IRQ_IN 0x84
#define USBTEST_EP_IRQ_OUT 0x3

#define USBTEST_CTRL_N_ENABLE
#define USBTEST_INTERFACE_CTRL 3
#define USBTEST_EP_CTRL_N 0x5

#define USBTEST_COUNT 8388608

//#define USBTEST_DEBUG

#define USBTEST_BUFFER_SIZE 2048

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

static bool lfsr_data_check(uint8_t *data, size_t len, uint32_t *sd)
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
      printf("Read error data[%d] = 0x%x ref = 0x%x\n", i, data[i], ref);
      return 1;
    }
  }

  *sd = seed;
  return 0;
}

static void print_devs(libusb_device **devs)
{
  libusb_device *dev;
  int i = 0;

  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
              fprintf(stderr, "failed to get device descriptor");
              return;
      }

      printf("%04x:%04x (bus %d, device %d)\n",
                  desc.idVendor, desc.idProduct,
                  libusb_get_bus_number(dev), libusb_get_device_address(dev));
    }
}

void cb(struct libusb_transfer *tr);

uint8_t valid_write_transfer(struct libusb_transfer *tr)
{
  if (tr->status != LIBUSB_TRANSFER_COMPLETED)
    {
      printf("Error transfer is not complete %s\n", libusb_error_name(tr->status));
      return 0;
    }

  uint32_t done = tr->actual_length;

  if (tr->type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) 
    {
      for (uint32_t i = 0; i < tr->num_iso_packets; i++) 
        {
          struct libusb_iso_packet_descriptor *d = &tr->iso_packet_desc[i];
   
            if (d->status != LIBUSB_TRANSFER_COMPLETED) 
              {
                printf("Error packet %d transfer error %d\n", i,  d->status);
                return 0;
              }
          
          if (d->length != d->actual_length)
            {
              printf("Write %d bytes on %d on packet %d\n", d->actual_length, d->length, i);
              return 0;
            }
          done += d->actual_length;
        }
    }

  if (tr->type == LIBUSB_TRANSFER_TYPE_CONTROL)
    done += 8; 
    
  if (done != tr->length)  
    {
      printf("Write %d bytes on %d\n", done, tr->length);
      return 0;
    }

  if (tr->type == LIBUSB_TRANSFER_TYPE_CONTROL)
    done -= 8;

  return 1;  
}

uint8_t valid_read_transfer(struct libusb_transfer *tr, uint32_t *last, uint8_t *buffer)
{
  if (tr->status != LIBUSB_TRANSFER_COMPLETED)
    {
      printf("Error transfer is not complete %s\n", libusb_error_name(tr->status));
      return 0;
    }

  if (tr->type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) 
    {
      for (uint32_t i = 0; i < tr->num_iso_packets; i++) 
        {
          struct libusb_iso_packet_descriptor *d = &tr->iso_packet_desc[i];
   
          if (d->status != LIBUSB_TRANSFER_COMPLETED) 
            {
              printf("Error packet %d transfer error %d\n", i,  d->status);
              return 0;
            }

          if (d->actual_length == 0)
            return 0;
          if (lfsr_data_check(buffer + (i * d->length) % USBTEST_BUFFER_SIZE,  d->actual_length, last))
            return 0;
        }
      return 1;
    }

  if (lfsr_data_check(buffer, tr->actual_length, last))
    return 0;

  return 1;  
}

#ifdef USBTEST_CTRL_0_ENABLE

uint32_t last0 = 0;

void ctrl_rdcb0(struct libusb_transfer *tr)
{
  if (valid_read_transfer(tr, &last0, tr->buffer + 8) == 0)
  {
    printf("Error while reading on endpoint 0 \n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

void ctrl_wrcb0(struct libusb_transfer *tr)
{
  if (valid_write_transfer(tr) == 0)
  {
    printf("error %d while writing on endpoint 0\n", tr->status);
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

#endif

#ifdef USBTEST_CTRL_N_ENABLE

uint32_t lastn = 0;

void ctrl_rdcbn(struct libusb_transfer *tr)
{
  if (valid_read_transfer(tr, &lastn, tr->buffer + 8) == 0)
  {
    printf("Error while reading on endpoint N \n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

void ctrl_wrcbn(struct libusb_transfer *tr)
{
  if (valid_write_transfer(tr) == 0)
  {
    printf("error %d while writing on endpoint N\n", tr->status);
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

#endif

#ifdef USBTEST_INTERFACE_BULK_ENABLE

uint32_t rlast0 = 0;
uint32_t wlast0 = 0;

void rdcb0(struct libusb_transfer *tr)
{
  if (valid_read_transfer(tr, &rlast0, tr->buffer) == 0)
  {
    printf("Error while reading on interface bulk \n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

void wrcb0(struct libusb_transfer *tr)
{
  if (valid_write_transfer(tr) == 0)
  {
    printf("Error while writing on interface bulk\n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

#endif

#ifdef USBTEST_INTERFACE_ISO_ENABLE

uint32_t rlast1 = 0;
uint32_t wlast1 = 0;

void rdcb1(struct libusb_transfer *tr)
{
  if (valid_read_transfer(tr, &rlast1, tr->buffer) == 0)
  {
    printf("Error while reading on interface iso\n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

void wrcb1(struct libusb_transfer *tr)
{
  if (valid_write_transfer(tr) == 0)
  {
    printf("Error while writing on interface iso\n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

#endif

#ifdef USBTEST_INTERFACE_IRQ_ENABLE

uint32_t rlast2 = 0;
uint32_t wlast2 = 0;

void rdcb2(struct libusb_transfer *tr)
{
  if (valid_read_transfer(tr, &rlast2, tr->buffer) == 0)
  {
    printf("Error while reading on interface irq \n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

void wrcb2(struct libusb_transfer *tr)
{
  if (valid_write_transfer(tr) == 0)
  {
    printf("Error while writing on interface irq\n");
    exit(EXIT_FAILURE);
  }

  uint8_t *done = (uint8_t *)tr->user_data;
  *done = 1;
}   

#endif

int main(void)
{
  libusb_device **devs;
  libusb_device_handle *usbdev;
  libusb_context *ctx = NULL; 
  ssize_t cnt;

  int r = libusb_init(&ctx); 

  if(r < 0) {
    printf("Init error\n");
    return 1;
  }
  
  libusb_set_debug(ctx, 3); 
  /* List of device */
  cnt = libusb_get_device_list(ctx, &devs);
  
  if(cnt < 0) {
    printf("Get device error\n");
    return 1;
  }

  /* Display device list */
  print_devs(devs);

  usbdev = libusb_open_device_with_vid_pid(ctx, 0x10eb, 0x0026);

  if(usbdev == NULL)
    printf("Cannot open device\n");
  else
    printf("Device openned\n");

  /* Set configuration */

//  libusb_set_configuration(usbdev, 0);
//  printf("Device unconfiguration done\n");

//  libusb_set_configuration(usbdev, 1);
//  printf("Device configuration done\n");

//  return 0;

#ifdef USBTEST_CTRL_0_ENABLE

  unsigned char buf0[USBTEST_BUFFER_SIZE + 8] __attribute__ ((aligned (2)));

  struct libusb_transfer *tr0;
  /* Work on ctrl 0 endpoint */
  tr0 = libusb_alloc_transfer(0);
  if (!tr0)
    printf("Error on allocating transfer\n");

  uint8_t d = 1;
  cnt = USBTEST_COUNT;
#endif

#ifdef USBTEST_CTRL_N_ENABLE
  if(libusb_detach_kernel_driver(usbdev, USBTEST_INTERFACE_CTRL) == 0)
    printf("Driver detached\n");

  /* Claim interface */
  r = libusb_claim_interface(usbdev, USBTEST_INTERFACE_CTRL); 
  if (r < 0)
    printf("Can not claim interface control\n");

  unsigned char bufn[USBTEST_BUFFER_SIZE + 8] __attribute__ ((aligned (2)));

  struct libusb_transfer *trn;
  /* Work on ctrl 0 endpoint */
  trn = libusb_alloc_transfer(0);
  if (!trn)
    printf("Error on allocating transfer\n");

  uint8_t dn = 1;
  size_t cntn = USBTEST_COUNT;
#endif

#ifdef USBTEST_INTERFACE_BULK_ENABLE
  if(libusb_detach_kernel_driver(usbdev, USBTEST_INTERFACE_BULK) == 0)
    printf("Driver detached\n");

  /* Claim interface */
  r = libusb_claim_interface(usbdev, USBTEST_INTERFACE_BULK); 
  if (r < 0)
    printf("Can not claim interface bulk\n");

  uint8_t r0, w0;

  uint8_t rx0[USBTEST_TRANSFER_SIZE];

  struct libusb_transfer *tr_rd0, *tr_wr0;

  tr_rd0 = libusb_alloc_transfer(0);
  tr_wr0 = libusb_alloc_transfer(0);

  if (!tr_rd0 || !tr_wr0)
    printf("Error on allocating transfer\n");

  unsigned char buffer0[USBTEST_BUFFER_SIZE];

  #ifdef USBTEST_INTERFACE_BULK_ALTERNATE
    r0 = w0 = 1;
    uint32_t cnt0 = USBTEST_COUNT/2;
    uint8_t alt = 0;
  #else
    r0 = w0 = 1;
    uint32_t rcnt0 = USBTEST_COUNT;
    uint32_t wcnt0 = USBTEST_COUNT;
  #endif
#endif

#ifdef USBTEST_INTERFACE_ISO_ENABLE
  if(libusb_detach_kernel_driver(usbdev, USBTEST_INTERFACE_ISO) == 0)
    printf("Driver detached\n");

  /* Claim interface */
  r = libusb_claim_interface(usbdev, USBTEST_INTERFACE_ISO); 
  if (r < 0)
    printf("Can not claim interface iso\n");

  unsigned char buffer1[USBTEST_BUFFER_SIZE];

  uint8_t r1, w1;
  r1 = w1 = 1;

  int num_iso = 16;

  struct libusb_transfer *tr_rd1, *tr_wr1;

  tr_rd1 = libusb_alloc_transfer(num_iso);
  tr_wr1 = libusb_alloc_transfer(num_iso);

  if (!tr_rd1 || !tr_wr1)
    printf("Error on allocating transfer\n");

  uint8_t rx1[USBTEST_BUFFER_SIZE];

  uint32_t rcnt1 = USBTEST_COUNT;
  uint32_t wcnt1 = USBTEST_COUNT;
#endif

#ifdef USBTEST_INTERFACE_IRQ_ENABLE
  if(libusb_detach_kernel_driver(usbdev, USBTEST_INTERFACE_IRQ) == 0)
    printf("Driver detached\n");

  r = libusb_claim_interface(usbdev, USBTEST_INTERFACE_IRQ); 
  if (r < 0)
    printf("Can not claim interface irq\n");

  uint8_t r2, w2;
  r2 = w2 = 1;

  struct libusb_transfer *tr_rd2, *tr_wr2;

  tr_rd2 = libusb_alloc_transfer(0);
  tr_wr2 = libusb_alloc_transfer(0);

  if (!tr_rd2 || !tr_wr2)
    printf("Error on allocating transfer\n");

  unsigned char buffer2[USBTEST_BUFFER_SIZE];

  uint8_t rx2[USBTEST_BUFFER_SIZE];

  uint32_t rcnt2 = USBTEST_COUNT;
  uint32_t wcnt2 = USBTEST_COUNT;
#endif

  uint32_t size;

  while(1)
    {
      bool end = 1;
#ifdef USBTEST_CTRL_0_ENABLE
      end &= (!cnt && d);

      if (cnt && d)
        {
          size = rand() % USBTEST_BUFFER_SIZE;
          uint32_t dir = rand() % 2;

          if (!size && !dir)
            size++;
       
          uint8_t type = LIBUSB_REQUEST_TYPE_CLASS |
                         LIBUSB_RECIPIENT_INTERFACE;
          if (dir)
            type |= LIBUSB_ENDPOINT_OUT;
          else
            type |= LIBUSB_ENDPOINT_IN;
  

#ifdef USBTEST_DEBUG
          printf("CTRL 0 size %d dir %d last %d\n", size, dir, last0);
#endif
          d = 0;
       
          libusb_fill_control_setup(buf0, type, 0, 0, 0, size);
       
          if (dir)
            {
              lfsr_data(buf0 + 8, size, &last0);

              libusb_fill_control_transfer(tr0, usbdev, buf0, &ctrl_wrcb0, &d, 0);
              tr0->actual_length = 0;
            }
          else
            libusb_fill_control_transfer(tr0, usbdev, buf0, &ctrl_rdcb0, &d, 0);
       
          libusb_submit_transfer(tr0);
          cnt--;
        }
#endif

#ifdef USBTEST_CTRL_N_ENABLE
      end &= (!cntn && dn);

      if (cntn && dn)
        {
          size = rand() % USBTEST_BUFFER_SIZE;
          uint32_t dir = rand() % 2;
       
          uint8_t type = 0;

          if (dir)
            type |= LIBUSB_ENDPOINT_OUT;
          else
            type |= LIBUSB_ENDPOINT_IN;
  
#ifdef USBTEST_DEBUG
        printf("CTRL N size %d dir %d last %d\n", size, dir, lastn);
#endif
          dn = 0;
       
          libusb_fill_control_setup(bufn, type, 0, 0, 0, size);
       
          if (dir)
            {
              lfsr_data(bufn + 8, size, &lastn);
              libusb_fill_control_transfer(trn, usbdev, bufn, &ctrl_wrcbn, &dn, 0);
            }
          else
            libusb_fill_control_transfer(trn, usbdev, bufn, &ctrl_rdcbn, &dn, 0);

          trn->endpoint = USBTEST_EP_CTRL_N,
       
          libusb_submit_transfer(trn);
          cntn--;
        }
#endif

#ifdef USBTEST_INTERFACE_BULK_ENABLE
  #ifdef USBTEST_INTERFACE_BULK_ALTERNATE
      end &= (!cnt0 && r0 && w0);
     
      if (cnt0 && r0 && w0)
        {
          alt = alt ^ 1;
         
          if (libusb_set_interface_alt_setting(usbdev, 0, alt))
            {
              printf("Error on setting alternate configuration\n");
              break;
            }
     
          r0 = w0 = 0;
     
          /* Push next transfer */
          if (alt)
            libusb_fill_bulk_transfer(tr_rd0, usbdev, USBTEST_EP_ALT_BULK_IN, rx0, USBTEST_TRANSFER_SIZE, &rdcb0, &r0, 0);
          else
            libusb_fill_bulk_transfer(tr_rd0, usbdev, USBTEST_EP_BULK_IN, rx0, USBTEST_TRANSFER_SIZE, &rdcb0, &r0, 0);

          libusb_submit_transfer(tr_rd0);
     
          size = rand() % USBTEST_BUFFER_SIZE;
     
          lfsr_data(buffer0, size, &wlast0);
          
          if (alt)
            libusb_fill_bulk_transfer(tr_wr0, usbdev, USBTEST_EP_ALT_BULK_OUT, buffer0, size, &wrcb0, &w0, 0);
          else
            libusb_fill_bulk_transfer(tr_wr0, usbdev, USBTEST_EP_BULK_OUT, buffer0, size, &wrcb0, &w0, 0);

          libusb_submit_transfer(tr_wr0);
          cnt0--;
        }
  #else
      end &= (!wcnt0 && !rcnt0 && r0 && w0);
 
      if (wcnt0 && w0)
        {
          wcnt0--;
          w0 = 0;
  
          size = rand() % USBTEST_BUFFER_SIZE;
  
#ifdef USBTEST_DEBUG
        printf("BULK OUT %d %d\n", size, wcnt0);
#endif
          lfsr_data(buffer0, size, &wlast0);
  
          /* Push next transfer */
          libusb_fill_bulk_transfer(tr_wr0, usbdev, USBTEST_EP_BULK_OUT, buffer0, size, &wrcb0, &w0, 0);
          if (libusb_submit_transfer(tr_wr0))
            printf("Error on submitting write transfer\n");
        }
      if (rcnt0 && r0)
        {
          rcnt0--;
          r0 = 0;
 
#ifdef USBTEST_DEBUG
        printf("BULK IN %d\n", rcnt0);
#endif
          /* Push next transfer */
          libusb_fill_bulk_transfer(tr_rd0, usbdev, USBTEST_EP_BULK_IN, rx0, USBTEST_TRANSFER_SIZE, &rdcb0, &r0, 0);
          if (libusb_submit_transfer(tr_rd0))
            printf("Error on submitting read transfer\n");
        }
  #endif
#endif

#ifdef USBTEST_INTERFACE_ISO_ENABLE
    end &= (!wcnt1 && !rcnt1 && r1 && w1);

    if (wcnt1 && w1)
      {
        wcnt1--;
        w1 = 0;
        size = rand() % USBTEST_BUFFER_SIZE;
     
        uint32_t pl = size/num_iso;
        lfsr_data(buffer1, pl * num_iso, &wlast1);

#ifdef USBTEST_DEBUG
        printf("ISO OUT %d %d\n", size, wcnt1);
#endif
        /* Push next transfer */
        libusb_fill_iso_transfer(tr_wr1, usbdev, USBTEST_EP_ISO_OUT, buffer1, pl * num_iso, num_iso, &wrcb1, &w1, 0);
        libusb_set_iso_packet_lengths(tr_wr1, pl);

        if (libusb_submit_transfer(tr_wr1))
          printf("Error on submitting write transfer\n");
      }
    if (rcnt1 && r1)
      {
        rcnt1--;
        r1 = 0;
#ifdef USBTEST_DEBUG
        printf("ISO IN %d\n", rcnt1);
#endif
        /* Push next transfer */
        libusb_fill_iso_transfer(tr_rd1, usbdev, USBTEST_EP_ISO_IN, rx1, USBTEST_BUFFER_SIZE, num_iso, &rdcb1, &r1, 0);
        libusb_set_iso_packet_lengths(tr_rd1, USBTEST_BUFFER_SIZE/num_iso);

        if (libusb_submit_transfer(tr_rd1))
          printf("Error on submitting read transfer\n");
      }
#endif

#ifdef USBTEST_INTERFACE_IRQ_ENABLE
    end &= (!wcnt2 && !rcnt2 && r2 && w2);

    if (wcnt2 && w2)
      {
        wcnt2--;
        w2 = 0;
        size = rand() % 64;

        lfsr_data(buffer2, size, &wlast2);

#ifdef USBTEST_DEBUG
        printf("IRQ OUT %d %d\n", size, wcnt2);
#endif
        /* Push next transfer */
        libusb_fill_interrupt_transfer(tr_wr2, usbdev, USBTEST_EP_IRQ_OUT, buffer2, size, &wrcb2, &w2, 0);
        if (libusb_submit_transfer(tr_wr2))
          printf("Error on submitting write transfer\n");
      }
    if (rcnt2 && r2)
      {
        rcnt2--;
        r2 = 0;
#ifdef USBTEST_DEBUG
        printf("IRQ IN %d\n", rcnt2);
#endif
        /* Push next transfer */
        libusb_fill_interrupt_transfer(tr_rd2, usbdev, USBTEST_EP_IRQ_IN, rx2, USBTEST_TRANSFER_SIZE, &rdcb2, &r2, 0);
        if (libusb_submit_transfer(tr_rd2))
          printf("Error on submitting read transfer\n");
      }
#endif
      if (end)
        break;

      if (libusb_handle_events(NULL))
        printf("Error on handling events\n");
    }


  printf("Done\n");

  /* Release interface and close device */
  libusb_free_device_list(devs, 1);
  libusb_close(usbdev); 
  libusb_exit(NULL);
  return 0;
}
