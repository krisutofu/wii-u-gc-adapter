// See LICENSE for license

#define _XOPEN_SOURCE 600

#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <getopt.h>

#include <linux/input.h>
#include <linux/uinput.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <libudev.h>
#include <libusb.h>
#include <pthread.h>

#if (!defined(LIBUSBX_API_VERSION) || LIBUSBX_API_VERSION < 0x01000102) && (!defined(LIBUSB_API_VERSION) || LIBUSB_API_VERSION < 0x01000102)
#error libusb(x) 1.0.16 or higher is required
#endif

#define USB_ID_VENDOR  0x057e
#define USB_ID_PRODUCT 0x0337

#define EP_IN  0x81
#define EP_OUT 0x02

#define STATE_NORMAL   0x10
#define STATE_WAVEBIRD 0x20

#define MAX_FF_EVENTS 4

const int BUTTON_OFFSET_VALUES[16] = {
   BTN_START,
   BTN_TR2,
   BTN_TR,
   BTN_TL,
   -1,
   -1,
   -1,
   -1,
   BTN_SOUTH,
   BTN_WEST,
   BTN_EAST,
   BTN_NORTH,
   BTN_DPAD_LEFT,
   BTN_DPAD_RIGHT,
   BTN_DPAD_DOWN,
   BTN_DPAD_UP,
};
const int BUTTON_ALTERNATIVE_VALUES[16] = {
   BTN_START,
   BTN_Z,
   BTN_TR,
   BTN_TL,
   -1,
   -1,
   -1,
   -1,
   BTN_A,
   BTN_B,
   BTN_X,
   BTN_Y,
   BTN_DPAD_LEFT,
   BTN_DPAD_RIGHT,
   BTN_DPAD_DOWN,
   BTN_DPAD_UP,
};

const int AXIS_OFFSET_VALUES[6] = {
   ABS_X,
   ABS_Y,
   ABS_RX,
   ABS_RY,
   ABS_Z,
   ABS_RZ
};

struct ff_event
{
   bool in_use;
   bool forever;
   int duration;
   int delay;
   int repetitions;
   struct timespec start_time;
   struct timespec end_time;
};

struct ports
{
   bool connected;
   bool extra_power;
   int uinput;
   unsigned char type;
   uint16_t buttons;
   uint8_t axis[6];
   struct ff_event ff_events[MAX_FF_EVENTS];
};

struct adapter
{
   volatile bool quitting;
   struct libusb_device *device;
   struct libusb_device_handle *handle;
   pthread_t thread;
   unsigned char rumble[5];
   struct ports controllers[4];
   struct adapter *next;
};

static bool uses_raw_mode;
static bool uses_abxyz_buttons = false;
static bool uses_trigger_or_shoulder = false;
static bool uses_trigger_as_shoulder = false;
static bool quits_on_interrupt = false;

static volatile int quitting;

static struct adapter adapters;

static const char *uinput_path;

static uint16_t vendor_id = USB_ID_VENDOR;

static uint16_t product_id = USB_ID_PRODUCT;

static unsigned char connected_type(unsigned char status)
{
   unsigned char type = status & (STATE_NORMAL | STATE_WAVEBIRD);
   switch (type)
   {
      case STATE_NORMAL:
      case STATE_WAVEBIRD:
         return type;
      default:
         return 0;
   }
}

static struct uinput_user_dev default_udev_settings = {
   .absmin[ABS_X]  = 35, .absmax[ABS_X]  = 220, .absfuzz[ABS_X]  = 2, .absflat[ABS_X] = 0,
   .absmin[ABS_Y]  = 35, .absmax[ABS_Y]  = 220, .absfuzz[ABS_Y]  = 2, .absflat[ABS_Y] = 0,
   .absmin[ABS_RX] = 35, .absmax[ABS_RX] = 218, .absfuzz[ABS_RX] = 2, .absflat[ABS_RX] = 0,
   .absmin[ABS_RY] = 35, .absmax[ABS_RY] = 218, .absfuzz[ABS_RY] = 2, .absflat[ABS_RY] = 0,
   .absmin[ABS_Z]  = 35, .absmax[ABS_Z]  = 190, .absfuzz[ABS_Z]  = 4, .absflat[ABS_Z] = 35,
   .absmin[ABS_RZ] = 35, .absmax[ABS_RZ] = 190, .absfuzz[ABS_RZ] = 4, .absflat[ABS_RZ] = 35,
};

static void set_raw_absinfo()
{
   uses_raw_mode = true;
   default_udev_settings.absmin[ABS_X]  = 0;  default_udev_settings.absmax[ABS_X]  = 255;
   default_udev_settings.absmin[ABS_Y]  = 0;  default_udev_settings.absmax[ABS_Y]  = 255;
   default_udev_settings.absmin[ABS_RX] = 0;  default_udev_settings.absmax[ABS_RX] = 255;
   default_udev_settings.absmin[ABS_RY] = 0;  default_udev_settings.absmax[ABS_RY] = 255;
   default_udev_settings.absmin[ABS_Z]  = 0;  default_udev_settings.absmax[ABS_Z]  = 255;
   default_udev_settings.absmin[ABS_RZ] = 0;  default_udev_settings.absmax[ABS_RZ] = 255;
}

/** Expects the command line settings to a comma separated list of assignments. The allowed variables are LX, LY, L, RX, RY, R and the allowed values are unsigned integer literals.
 *  The array_offset must be the offsetof(struct uinput_user_dev, …) of an array member field.
  */
static void set_axis_absinfo(ptrdiff_t array_offset, char command_line_settings[])
{
   signed int *absinfo_array = (signed int*)((char*) &default_udev_settings + array_offset);

   for ( char *next_item = strtok(command_line_settings, ","); next_item != NULL; next_item = strtok(NULL, ",") )
   {
      int axis_name_length = strcspn(next_item, "=");
      if (next_item[axis_name_length] == '\0')
         continue;

      char *axis_value_string = next_item + axis_name_length + 1;
      char *axis_value_end;
      int axis_value = strtoul(axis_value_string, &axis_value_end, 10);
      if (axis_value_string == axis_value_end)
         continue;

      if (next_item[0] == 'L')
      {
         if (next_item[1] == 'X')
            absinfo_array[ABS_X] = axis_value;
         else if (next_item[1] == 'Y')
            absinfo_array[ABS_Y] = axis_value;
         else
            absinfo_array[ABS_Z] = axis_value;
      }
      else if (next_item[0] == 'R')
      {
         if (next_item[1] == 'X')
            absinfo_array[ABS_RX] = axis_value;
         else if (next_item[1] == 'Y')
            absinfo_array[ABS_RY] = axis_value;
         else
            absinfo_array[ABS_RZ] = axis_value;
      }
   }
}

static bool uinput_create(int i, struct ports *port, unsigned char type)
{
   fprintf(stderr, "connecting on port %d\n", i);
   struct uinput_user_dev uinput_dev = default_udev_settings;
   port->uinput = open(uinput_path, O_RDWR | O_NONBLOCK);

   // buttons
   ioctl(port->uinput, UI_SET_EVBIT, EV_KEY);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_NORTH);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_SOUTH);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_EAST);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_WEST);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_START);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_DPAD_UP);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_DPAD_DOWN);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_DPAD_LEFT);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_DPAD_RIGHT);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_TL);
   ioctl(port->uinput, UI_SET_KEYBIT, BTN_TR);

   if (uses_abxyz_buttons)
      ioctl(port->uinput, UI_SET_KEYBIT, BTN_Z);
   else
      ioctl(port->uinput, UI_SET_KEYBIT, BTN_TR2);

   // axis
   ioctl(port->uinput, UI_SET_EVBIT, EV_ABS);
   ioctl(port->uinput, UI_SET_ABSBIT, ABS_X);
   ioctl(port->uinput, UI_SET_ABSBIT, ABS_Y);
   ioctl(port->uinput, UI_SET_ABSBIT, ABS_RX);
   ioctl(port->uinput, UI_SET_ABSBIT, ABS_RY);
   ioctl(port->uinput, UI_SET_ABSBIT, ABS_Z);
   ioctl(port->uinput, UI_SET_ABSBIT, ABS_RZ);

   // rumble
   ioctl(port->uinput, UI_SET_EVBIT, EV_FF);
   ioctl(port->uinput, UI_SET_FFBIT, FF_PERIODIC);
   ioctl(port->uinput, UI_SET_FFBIT, FF_SQUARE);
   ioctl(port->uinput, UI_SET_FFBIT, FF_TRIANGLE);
   ioctl(port->uinput, UI_SET_FFBIT, FF_SINE);
   ioctl(port->uinput, UI_SET_FFBIT, FF_RUMBLE);
   uinput_dev.ff_effects_max = MAX_FF_EVENTS;

   snprintf(uinput_dev.name, sizeof(uinput_dev.name), "Wii U GameCube Adapter Port %d", i+1);
   uinput_dev.name[sizeof(uinput_dev.name)-1] = 0;
   uinput_dev.id.bustype = BUS_USB;
   uinput_dev.id.vendor = vendor_id;
   uinput_dev.id.product = product_id;

   size_t to_write = sizeof(uinput_dev);
   size_t written = 0;
   while (written < to_write)
   {
      ssize_t write_ret = write(port->uinput, (const char*)&uinput_dev + written, to_write - written);
      if (write_ret < 0)
      {
         perror("error writing uinput device settings");
         close(port->uinput);
         return false;
      }
      written += write_ret;
   }

   if (ioctl(port->uinput, UI_DEV_CREATE) != 0)
   {
      perror("error creating uinput device");
      close(port->uinput);
      return false;
   }
   port->type = type;
   port->connected = true;
   return true;
}

static void uinput_destroy(int i, struct ports *port)
{
   fprintf(stderr, "disconnecting on port %d\n", i);
   ioctl(port->uinput, UI_DEV_DESTROY);
   close(port->uinput);
   port->connected = false;
}

static struct timespec ts_add(struct timespec *start, int milliseconds)
{
   struct timespec ret = *start;
   int s = milliseconds / 1000;
   int ns = (milliseconds % 1000) * 1000000;
   ret.tv_sec += s ;
   ret.tv_nsec += ns ;
   if (ret.tv_nsec >= 1000000000L)
   {
      ret.tv_sec++;
      ret.tv_nsec -= 1000000000L;
   }
   return ret;
}

static bool ts_greaterthan(struct timespec *first, struct timespec *second)
{
   return (first->tv_sec >= second->tv_sec || (first->tv_sec == second->tv_sec && first->tv_nsec >= second->tv_nsec));
}

static bool ts_lessthan(struct timespec *first, struct timespec *second)
{
   return (first->tv_sec <= second->tv_sec || (first->tv_sec == second->tv_sec && first->tv_nsec <= second->tv_nsec));
}

static void update_ff_start_stop(struct ff_event *e, struct timespec *current_time)
{
   e->repetitions--;

   if (e->repetitions < 0)
   {
      e->repetitions = 0;
      e->start_time.tv_sec = 0;
      e->start_time.tv_nsec = 0;
      e->end_time.tv_sec = 0;
      e->end_time.tv_nsec = 0;
   }
   else
   {
      e->start_time = ts_add(current_time, e->delay);
      if (e->forever)
      {
         e->end_time.tv_sec = INT_MAX;
         e->end_time.tv_nsec = 999999999L;
      }
      else
      {
         e->end_time = ts_add(&e->start_time, e->duration);
      }
   }
}

static int create_ff_event(struct ports *port, struct uinput_ff_upload *upload)
{
   bool stop = false;
   switch (upload->effect.type)
   {
      case FF_PERIODIC:
         stop = upload->effect.u.periodic.magnitude == 0;
         break;
      case FF_RUMBLE:
         stop = upload->effect.u.rumble.strong_magnitude == 0 && upload->effect.u.rumble.weak_magnitude == 0;
         break;
   }
   if (upload->old.type != 0)
   {
      if (stop)
      {
         port->ff_events[upload->old.id].forever = false;
         port->ff_events[upload->old.id].duration = 0;
      }
      else
      {
         // events with length 0 last forever
         port->ff_events[upload->old.id].forever = (upload->effect.replay.length == 0);
         port->ff_events[upload->old.id].duration = upload->effect.replay.length;
      }
      port->ff_events[upload->old.id].delay = upload->effect.replay.delay;
      port->ff_events[upload->old.id].repetitions = 0;
      return upload->old.id;
   }
   for (int i = 0; i < MAX_FF_EVENTS; i++)
   {
      if (!port->ff_events[i].in_use)
      {
         port->ff_events[i].in_use = true;
         if (stop)
         {
            port->ff_events[i].forever = false;
            port->ff_events[i].duration = 0;
         }
         else
         {
            port->ff_events[i].forever = (upload->effect.replay.length == 0);
            port->ff_events[i].duration = upload->effect.replay.length;
         }
         port->ff_events[i].delay = upload->effect.replay.delay;
         port->ff_events[i].repetitions = 0;
         return i;
      }
   }

   return -1;
}

static void add_button_event(struct input_event events[], int *events_count, uint16_t previous_button_state, uint16_t *result_button_state, const int button_codes[], uint16_t button_pressed_mask, int tested_button_id)
{
   int button_code = button_codes[tested_button_id];
   if (button_code == -1)
      return;

   uint16_t single_button_mask = (1 << tested_button_id);
   uint16_t single_button_pressed_mask = button_pressed_mask & single_button_mask;


   if ((previous_button_state & single_button_mask) != single_button_pressed_mask)
   {
      int e_count = *events_count;
      events[e_count].type = EV_KEY;
      events[e_count].code = button_code;
      events[e_count].value = (single_button_pressed_mask != 0);
      e_count++;
      *events_count = e_count;

      previous_button_state &= ~single_button_mask;
      previous_button_state |= single_button_pressed_mask;
      *result_button_state = previous_button_state;
   }
}

static void handle_payload(int i, struct ports *port, unsigned char *payload, struct timespec *current_time)
{
   unsigned char status = payload[0];
   unsigned char type = connected_type(status);

   if (type != 0 && !port->connected)
   {
      uinput_create(i, port, type);
   }
   else if (type == 0 && port->connected)
   {
      uinput_destroy(i, port);
   }

   if (!port->connected)
      return;

   port->extra_power = ((status & 0x04) != 0);

   if (type != port->type)
   {
      fprintf(stderr, "controller on port %d changed controller type???\n", i+1);
      port->type = type;
   }

   struct input_event events[16+6+1] = {0}; // buttons + axis + syn event
   int e_count = 0;

   uint16_t btns = (uint16_t) payload[1] << 8 | (uint16_t) payload[2];

   uint16_t previous_buttons_state = port->buttons;
   for (int j = 0; j < 16; j++)
   {
      if (uses_abxyz_buttons)
      {
         add_button_event(events, &e_count, previous_buttons_state, &port->buttons, BUTTON_ALTERNATIVE_VALUES, btns, j);
      }
      else
      {
         add_button_event(events, &e_count, previous_buttons_state, &port->buttons, BUTTON_OFFSET_VALUES, btns, j);
      }
   }

   previous_buttons_state = (previous_buttons_state ^ port->buttons) & port->buttons;
   int left_shoulder_index = 3;
   bool is_left_shoulder_pressed_down = previous_buttons_state & (1 << left_shoulder_index);
   int right_shoulder_index = 2;
   bool is_right_shoulder_pressed_down =  previous_buttons_state & (1 << right_shoulder_index);

   for (int j = 0; j < 6; j++)
   {
      unsigned char value = payload[j+3];
      int current_axis = AXIS_OFFSET_VALUES[j];

      if (current_axis == ABS_Y || current_axis == ABS_RY)
         value ^= 0xFF; // flip from 0 - 255 to 255 - 0
      else if (uses_trigger_as_shoulder && (current_axis == ABS_Z || current_axis == ABS_RZ))
      {
         value = value > default_udev_settings.absmin[current_axis] + 20;
         int button_mask = 1 << ((current_axis == ABS_Z) ? 3 : 2);

         if ((port->buttons & button_mask) != value)
         {
            events[e_count].type = EV_KEY;
            events[e_count].code = (current_axis == ABS_Z) ? BTN_TL : BTN_TR;
            events[e_count].value = value;
            e_count++;
            port->buttons &= ~button_mask;
            port->buttons |= button_mask * value;
         }
         continue;
      }
      else if (uses_trigger_or_shoulder)
      {
         if (is_left_shoulder_pressed_down && current_axis == ABS_Z)
            value = default_udev_settings.absmin[ABS_Z];
         else if (is_right_shoulder_pressed_down && current_axis == ABS_RZ)
            value = default_udev_settings.absmin[ABS_RZ];
      }

      if (port->axis[j] != value)
      {
         events[e_count].type = EV_ABS;
         events[e_count].code = current_axis;
         events[e_count].value = value;
         e_count++;
         port->axis[j] = value;
      }
   }

   if (e_count > 0)
   {
      events[e_count].type = EV_SYN;
      events[e_count].code = SYN_REPORT;
      e_count++;
      size_t to_write = sizeof(events[0]) * e_count;
      size_t written = 0;
      while (written < to_write)
      {
         ssize_t write_ret = write(port->uinput, (const char*)events + written, to_write - written);
         if (write_ret < 0)
         {
            perror("Warning: writing input events failed");
            break;
         }
         written += write_ret;
      }
   }

   // check for rumble events
   struct input_event e;
   ssize_t ret = read(port->uinput, &e, sizeof(e));
   if (ret == sizeof(e))
   {
      if (e.type == EV_UINPUT)
      {
         switch (e.code)
         {
            case UI_FF_UPLOAD:
            {
               struct uinput_ff_upload upload = { 0 };
               upload.request_id = e.value;
               ioctl(port->uinput, UI_BEGIN_FF_UPLOAD, &upload);
               int id = create_ff_event(port, &upload);
               if (id < 0)
               {
                  // TODO: what's the proper error code for this?
                  upload.retval = -1;
               }
               else
               {
                  upload.retval = 0;
                  upload.effect.id = id;
               }
               ioctl(port->uinput, UI_END_FF_UPLOAD, &upload);
               break;
            }
            case UI_FF_ERASE:
            {
               struct uinput_ff_erase erase = { 0 };
               erase.request_id = e.value;
               ioctl(port->uinput, UI_BEGIN_FF_ERASE, &erase);
               if (erase.effect_id < MAX_FF_EVENTS)
                  port->ff_events[erase.effect_id].in_use = false;
               ioctl(port->uinput, UI_END_FF_ERASE, &erase);
            }
         }
      }
      else if (e.type == EV_FF)
      {
         if (e.code < MAX_FF_EVENTS && port->ff_events[e.code].in_use)
         {
            port->ff_events[e.code].repetitions = e.value;
            update_ff_start_stop(&port->ff_events[e.code], current_time);
         }
      }
   }
}

static void *adapter_thread(void *data)
{
   struct adapter *a = (struct adapter *)data;

    int bytes_transferred;
    unsigned char payload[1] = { 0x13 };

    int transfer_ret = libusb_interrupt_transfer(a->handle, EP_OUT, payload, sizeof(payload), &bytes_transferred, 0);

    if (transfer_ret != 0) {
        fprintf(stderr, "libusb_interrupt_transfer: %s\n", libusb_error_name(transfer_ret));
        return NULL;
    }
    if (bytes_transferred != sizeof(payload)) {
        fprintf(stderr, "libusb_interrupt_transfer %d/%d bytes transferred.\n", bytes_transferred, sizeof(payload));
        return NULL;
    }

   #define decide_on_quitting_the_loop() do { \
         if (quits_on_interrupt) { \
            a->quitting = true; \
            break; \
         } \
   \
         sleep(1); \
   } while(0)

   while (!a->quitting)
   {
      unsigned char payload[37];
      int size = 0;
      int transfer_ret = libusb_interrupt_transfer(a->handle, EP_IN, payload, sizeof(payload), &size, 0);
      if (transfer_ret != 0) {
         fprintf(stderr, "libusb_interrupt_transfer error %d\n", transfer_ret);
         decide_on_quitting_the_loop();
         continue;
      }
      if (size != 37 || payload[0] != 0x21)
         continue;

      unsigned char *controller = &payload[1];

      unsigned char rumble[5] = { 0x11, 0, 0, 0, 0 };
      struct timespec current_time = { 0 };
      clock_gettime(CLOCK_MONOTONIC_RAW, &current_time);
      for (int i = 0; i < 4; i++, controller += 9)
      {
         handle_payload(i, &a->controllers[i], controller, &current_time);
         rumble[i+1] = 0;
         if (a->controllers[i].extra_power && a->controllers[i].type == STATE_NORMAL)
         {
            for (int j = 0; j < MAX_FF_EVENTS; j++)
            {
               struct ff_event *e = &a->controllers[i].ff_events[j];
               if (e->in_use)
               {
                  bool after_start = ts_lessthan(&e->start_time, &current_time);
                  bool before_end = ts_greaterthan(&e->end_time, &current_time);

                  if (after_start && before_end)
                     rumble[i+1] = 1;
                  else if (after_start && !before_end)
                     update_ff_start_stop(e, &current_time);
               }
            }
         }
      }

      if (memcmp(rumble, a->rumble, sizeof(rumble)) != 0)
      {
         memcpy(a->rumble, rumble, sizeof(rumble));
         transfer_ret = libusb_interrupt_transfer(a->handle, EP_OUT, a->rumble, sizeof(a->rumble), &size, 0);
         if (transfer_ret != 0) {
            fprintf(stderr, "libusb_interrupt_transfer error %d\n", transfer_ret);
            decide_on_quitting_the_loop();
            continue;
         }
      }
   }

   for (int i = 0; i < 4; i++)
   {
      if (a->controllers[i].connected)
         uinput_destroy(i, &a->controllers[i]);
   }

   return NULL;
}

static void add_adapter(struct libusb_device *dev)
{
   struct adapter *a = calloc(1, sizeof(struct adapter));
   if (a == NULL)
   {
      fprintf(stderr, "FATAL: calloc() failed\n");
      exit(-1);
   }
   a->device = dev;

   if (libusb_open(a->device, &a->handle) != 0)
   {
      fprintf(stderr, "Error opening device %p\n", a->device);
      return;
   }

   if (libusb_kernel_driver_active(a->handle, 0) == 1) {
       fprintf(stderr, "Detaching kernel driver\n");
       if (libusb_detach_kernel_driver(a->handle, 0)) {
           fprintf(stderr, "Error detaching handle %p from kernel\n", a->handle);
           return;
       }
   }

   struct adapter *old_head = adapters.next;
   adapters.next = a;
   a->next = old_head;

   pthread_create(&a->thread, NULL, adapter_thread, a);

   fprintf(stderr, "adapter %p connected\n", a->device);
}

static void remove_adapter(struct libusb_device *dev)
{
   struct adapter *a = &adapters;
   while (a->next != NULL)
   {
      if (a->next->device == dev)
      {
         a->next->quitting = true;
         pthread_join(a->next->thread, NULL);
         fprintf(stderr, "adapter %p disconnected\n", a->next->device);
         libusb_close(a->next->handle);
         struct adapter *new_next = a->next->next;
         free(a->next);
         a->next = new_next;
         return;
      }

      a = a->next;
   }
}

static int LIBUSB_CALL hotplug_callback(struct libusb_context *ctx, struct libusb_device *dev, libusb_hotplug_event event, void *user_data)
{
   (void)ctx;
   (void)user_data;
   if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
   {
      add_adapter(dev);
   }
   else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT)
   {
      remove_adapter(dev);
   }

   return 0;
}

static void quitting_signal(int sig)
{
   (void)sig;
   quitting = 1;
}

static uint16_t parse_id(const char* str)
{
   char* endptr = NULL;
   unsigned long out = strtoul(str, &endptr, 0);

   if (out > 0xffff || *endptr)
   {
      fprintf(stderr, "Invalid ID \"%s\"\n", str);
      exit(1);
   }

   return out;
}

enum {
   opt_vendor = 1000,
   opt_product,
   opt_use_abxyz_buttons,
   opt_only_abxyz_buttons,
   opt_quit_interrupt,
   opt_deadzone,
   opt_tolerance,
   opt_min,
   opt_max,
   opt_trigger_or_shoulder,
   opt_trigger_as_shoulder,
};

static struct option options[] = {
   { "help", no_argument, 0, 'h' },
   { "raw", no_argument, 0, 'r' },
   { "vendor", required_argument, 0, opt_vendor },
   { "product", required_argument, 0, opt_product },
   { "enable-abxyz", no_argument, 0, opt_use_abxyz_buttons },
   { "quit-on-interrupt", no_argument, 0, opt_quit_interrupt },
   { "deadzone", required_argument, 0, opt_deadzone },
   { "change-tolerance", required_argument, 0, opt_tolerance },
   { "min-value", required_argument, 0, opt_min },
   { "max-value", required_argument, 0, opt_max },
   { "shoulder-nand-trigger", no_argument, 0, opt_trigger_or_shoulder },
   { "trigger-buttons", no_argument, 0, opt_trigger_as_shoulder },
   { 0, 0, 0, 0 },
};

int main(int argc, char *argv[])
{
   struct udev *udev;
   struct udev_device *uinput;
   struct sigaction sa;

   memset(&sa, 0, sizeof(sa));

   while (1) {
      int option_index = 0;
      int c = getopt_long(argc, argv, "rh", options, &option_index);
      if (c == -1)
         break;

      if (c == 'h') {
         fprintf(stdout,
            "usage: wii-u-gc-adapter  [--help] [--raw] [--vendor ⟨int⟩] [--product ⟨int⟩] [--enable-abxyz] [--shoulder-nand-trigger] [--quit-on-interrupt]\\\n"
            "                 [--trigger-buttons]\\\n"
            "                 [--deadzone [LX=⟨int⟩,][LY=⟨int⟩,][RX=⟨int⟩,][RY=⟨int⟩,][L=⟨int⟩,][R=⟨int⟩,]\"\"] \\\n"
            "                 [--change-tolerance [LX=⟨int⟩,][LY=⟨int⟩,][RX=⟨int⟩,][RY=⟨int⟩,][L=⟨int⟩,][R=⟨int⟩,]\"\"] \\\n"
            "                 [--min-value [LX=⟨int⟩,][LY=⟨int⟩,][RX=⟨int⟩,][RY=⟨int⟩,][L=⟨int⟩,][R=⟨int⟩,]\"\"] \\\n"
            "                 [--max-value [LX=⟨int⟩,][LY=⟨int⟩,][RX=⟨int⟩,][RY=⟨int⟩,][L=⟨int⟩,][R=⟨int⟩,]\"\"] \\\n"
            "\n"
            "--raw removes the lower and upper limit in the analog input values, i.e. it sets min = 0, max = 255 instead of using the adjusted default values.\n"
            "--quit-on-interrupt will make the thread stop and exit when a libusb interrupt occurs, otherwise it tries to wait and retry.\n"
            "--vendor and --product correspond to the IDs associated to the event device that should be read. Default values are vendor = %p, product = %p.\n"
            "--enable-abxyz replaces the existing BTN_SOUTH, BTN_WEST, BTN_EAST, BTN_NORTH, BTN_TR2 events with BTN_A, BTN_B, BTN_X, BTN_Y, BTN_Z\n"
            "--shoulder-nand-trigger ensures that shoulder and trigger are not active together at the same time. As long as pressing the shoulder, the trigger is released.\n"
            "--trigger-buttons makes the triggers behave as shoulder buttons only. Use --min-value L=255,R=255 to disable the analog triggers entirely.\n"
            "--deadzone, --change-tolerance, --min-value and --max-value configure the analog axis event value.\n"
            "       Deadzone specifies a limit on the absolute value of the analog control element which suppresses events for smaller values, default value is '35' for L and R triggers.\n"
            "       The change tolerance specifies a limit on the value difference of the analog value which suppresses events for smaller differences, default value is '1'\n"
            "       Min Value is the lowest analog value emitted from an analog axis.\n"
            "       Max Value is the maximum analog value emitted from an analog axis. Note, it's slightly reduced for the left thumb stick and might require a gain for some games\n",
            USB_ID_VENDOR, USB_ID_PRODUCT
         );
         exit(0);
      }

      switch (c) {
      case 'r':
         fprintf(stderr, "raw mode enabled\n");
         set_raw_absinfo();
         break;
      case opt_vendor:
         vendor_id = parse_id(optarg);
         fprintf(stderr, "vendor_id = %#06x\n", vendor_id);
         break;
      case opt_product:
         product_id = parse_id(optarg);
         fprintf(stderr, "product_id = %#06x\n", product_id);
         break;
      case opt_use_abxyz_buttons: uses_abxyz_buttons = true; break;
      case opt_trigger_or_shoulder: uses_trigger_or_shoulder = true; break;
      case opt_quit_interrupt: quits_on_interrupt = true; break;
      case opt_deadzone: set_axis_absinfo(offsetof(struct uinput_user_dev, absflat), optarg); break;
      case opt_tolerance: set_axis_absinfo(offsetof(struct uinput_user_dev, absfuzz), optarg); break;
      case opt_min: set_axis_absinfo(offsetof(struct uinput_user_dev, absmin), optarg); break;
      case opt_max: set_axis_absinfo(offsetof(struct uinput_user_dev, absmax), optarg); break;
      case opt_trigger_as_shoulder:
         uses_trigger_as_shoulder = true;
         default_udev_settings.absmin[ABS_Z] -= 10;
         default_udev_settings.absmin[ABS_RZ] -= 10;
         break;
      }
   }

   sa.sa_handler = quitting_signal;
   sa.sa_flags = SA_RESTART | SA_RESETHAND;
   sigemptyset(&sa.sa_mask);

   sigaction(SIGINT, &sa, NULL);
   sigaction(SIGTERM, &sa, NULL);

   udev = udev_new();
   if (udev == NULL) {
      fprintf(stderr, "udev init errors\n");
      return -1;
   }

   uinput = udev_device_new_from_subsystem_sysname(udev, "misc", "uinput");
   if (uinput == NULL)
   {
      fprintf(stderr, "uinput creation failed\n");
      return -1;
   }

   uinput_path = udev_device_get_devnode(uinput);
   if (uinput_path == NULL)
   {
      fprintf(stderr, "cannot find path to uinput\n");
      return -1;
   }

   libusb_init(NULL);

   struct libusb_device **devices;

   int count = libusb_get_device_list(NULL, &devices);

   for (int i = 0; i < count; i++)
   {
      struct libusb_device_descriptor desc;
      libusb_get_device_descriptor(devices[i], &desc);
      if (desc.idVendor == USB_ID_VENDOR && desc.idProduct == USB_ID_PRODUCT)
         add_adapter(devices[i]);
   }

   if (count > 0)
      libusb_free_device_list(devices, 1);

   libusb_hotplug_callback_handle callback;

   int hotplug_capability = libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG);
   if (hotplug_capability) {
       int hotplug_ret = libusb_hotplug_register_callback(NULL,
             LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
             0, USB_ID_VENDOR, USB_ID_PRODUCT,
             LIBUSB_HOTPLUG_MATCH_ANY, hotplug_callback, NULL, &callback);

       if (hotplug_ret != LIBUSB_SUCCESS) {
           fprintf(stderr, "cannot register hotplug callback, hotplugging not enabled\n");
           hotplug_capability = 0;
       }
   }

   // pump events until shutdown & all helper threads finish cleaning up
   while (!quitting)
      libusb_handle_events_completed(NULL, (int *)&quitting);

   while (adapters.next)
      remove_adapter(adapters.next->device);

   if (hotplug_capability)
      libusb_hotplug_deregister_callback(NULL, callback);

   libusb_exit(NULL);
   udev_device_unref(uinput);
   udev_unref(udev);
   return 0;
}
