// See LICENSE for license

#define _XOPEN_SOURCE 600

#include <time.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <ctype.h>
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

// see here https://gist.github.com/nondebug/aec93dff7f0f1969f4cc2291b24a3171
enum VendorId {
   USB_NINTENDO_VENDOR = 0x057e,
   USB_MICROSOFT_VENDOR = 0x045e,
};
enum ProductId {
   USB_ID_PRODUCT = 0x0337,
   USB_XBOX360_PRODUCT = 0x028e,
   USB_XBOX360_WIRELESS_PRODUCT = 0x02a1,
   USB_XBOX_WIRELESS_USB_PRODUCT = 0x0b12,
   //USB_XBOX_WIRELESS_BLUETOOTH_PRODUCT = 0x0b13,
   //USB_XBOX_USA_PRODUCT = 0x0202,
   //USB_XBOX_USA_2_PRODUCT = 0x0289,
   //USB_XBOX_JAPAN_PRODUCT = 0x0285,
   USB_XBOX_S_PRODUCT = 0x0287,
   //USB_XBOX_S_2_PRODUCT = 0x0288,
   USB_XBOX_ONE_PRODUCT = 0x02d1,
   USB_XBOX_ONE_2_PRODUCT = 0x02dd,
   USB_XBOX_ONE_S_USB_PRODUCT = 0x02ea,
   //USB_XBOX_ONE_S_BLUETOOTH_PRODUCT = 0x02e0,
   //USB_XBOX_ONE_S_BLUETOOTH_2_PRODUCT = 0x02fd,
   USB_XBOX_ONE_ELITE_PRODUCT = 0x02e3,
   //USB_XBOX_ONE_ELITE_2_PRODUCT = 0x02ff,
   USB_XBOX_ONE_ELITE_SERIES_2_USB_PRODUCT = 0x0b00,
   USB_XBOX_ONE_ELITE_SERIES_2_PRODUCT = 0x0b05,
   //USB_XBOX_ONE_ELITE_SERIES_2_2_PRODUCT = 0x0b22,
};

#define EP_IN  0x81
#define EP_OUT 0x02

#define STATE_NORMAL   0x10
#define STATE_WAVEBIRD 0x20

#define MAX_FF_EVENTS 4

enum ButtonCodeIndex {
   start_button_index = 0,
   z_button_index = 1,
   r_button_index = 2,
   l_button_index = 3,
   a_button_index = 8,
   b_button_index = 9,
   x_button_index = 10,
   y_button_index = 11,
   left_button_index = 12,
   right_button_index = 13,
   down_button_index = 14,
   up_button_index = 15,
};

const int BUTTON_XBOX_VALUES[16] = {
   BTN_START,
   BTN_THUMBL,
   BTN_TR2,
   BTN_TL2,
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
const int BUTTON_LITERAL_VALUES[16] = {
   BTN_START,
   BTN_THUMBL,
   BTN_TR2,
   BTN_TL2,
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
const int REMAPPED_DPAD_DEFAULTS[4] = {
   BTN_TL,
   BTN_TR,
   BTN_THUMBR,
   BTN_SELECT,
};
static int dpad_button_codes[] = {
   BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_DPAD_DOWN, BTN_DPAD_UP,
};

#define BUTTON_COUNT 16
static int button_code_values[BUTTON_COUNT];

enum AxisInputIndex {
   thumbl_x_index,
   thumbl_y_index,
   thumbr_x_index,
   thumbr_y_index,
   trigger_l_index,
   trigger_r_index,
};

// also see opt_default_axes_map
#define AXIS_COUNT 6
static struct AxisCode {
   int lo;
   int hi;
} axis_code_values[AXIS_COUNT] = {
   { -1, ABS_X, },
   { -1, ABS_Y, },
   { -1, ABS_RX, },
   { -1, ABS_RY, },
   { -1, ABS_Z, },
   { -1, ABS_RZ, },
};

enum AxisDivision {
   lower_half_axis = -1,
   full_axis = 0,
   upper_half_axis = 1,
};


// remembers the controller's actual value ranges from each analog input
// each controller likely varies in which values it reaches so this values are wider than my platinum controller
static struct AxisRange {
   int min;
   int max;
} axis_natural_ranges[AXIS_COUNT] = {
   { 35, 218 },
   { 35, 218 },
   { 40, 215 },
   { 40, 215 },
   { 35, 230 }, // 230 is the maximum when fully depressed, 200 is reached by triggers before hitting the shoulder button
   { 35, 230 },
};

static int trigger_buttons[] = {
   BTN_TL2, BTN_TR2,
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

// parsed from command line options

static enum ShoulderButtonMode {
   shoulder_button_none,
   shoulder_button_nand,
   shoulder_button_and,
} uses_shoulder_button = shoulder_button_none;

static enum ThumbstickMode {
   thumbstick_none,
   thumbstick_normal,
   thumbstick_dpad,
   thumbstick_dpad_sensitive,
   thumbstick_analog_dpad,
   thumbstick_analog_dpad_flipped,
} uses_thumbstick_left = thumbstick_normal,
  uses_thumbstick_right = thumbstick_normal;

static enum TriggerMode {
   trigger_none,
   trigger_normal,
   trigger_binary,
} uses_trigger_left = trigger_normal,
  uses_trigger_right = trigger_normal;

static bool uses_explicit_libusb_claim = false;
static bool uses_raw_mode = false;
static bool flips_y_axis = true;
static bool uses_remapped_dpad = false;
static bool uses_foreign_buttons = false;
static bool quits_on_interrupt = false;
#define DEFAULT_Z_CODE BTN_THUMBL
static int z_code = DEFAULT_Z_CODE;

static volatile int quitting;

static struct adapter adapters;

static const char *uinput_path;

static uint16_t vendor_id = 0;

static uint16_t product_id = 0;

static const char *device_name = NULL;

enum ControllerId {
   gcn_adapter_index,
   xbox_360_index,
   xbox_360_wireless_index,
   xbox_wireless_index,
   xbox_s_index,
   xobx_one_index,
   xbox_one_2_index,
   xbox_one_s_index,
   xbox_one_elite_index,
   xobx_one_elite_series_2_index,
   xobx_one_elite_series_2_2_index,
   no_controller_index,
} controller_index = gcn_adapter_index;

// see PCGamingWiki, e.g. https://www.pcgamingwiki.com/wiki/Controller:Xbox_Wireless_Controller
static struct DeviceInfo {
   uint16_t vendor_id;
   uint16_t product_id;
   const char *device_name;
   bool flips_y_axis;
} device_data[] = {
   [gcn_adapter_index] = { .vendor_id = USB_NINTENDO_VENDOR, .product_id = USB_ID_PRODUCT, .device_name = "Wii U GameCube Adapter Port %d", .flips_y_axis = true },
   [xbox_360_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX360_PRODUCT, .device_name = "Microsoft X-Box 360 pad", .flips_y_axis = false },
   [xbox_360_wireless_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX360_WIRELESS_PRODUCT, .device_name = "Xbox 360 Wireless Receiver", .flips_y_axis = false },
   [xbox_wireless_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_WIRELESS_USB_PRODUCT, .device_name = "Xbox Wireless Controller", .flips_y_axis = false },
   //[xbox_wireless_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_WIRELESS_BLUETOOTH_PRODUCT, .device_name = "Xbox Wireless Controller", .flips_y_axis = false },
   [xbox_s_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_S_PRODUCT, .device_name = "Microsoft Xbox Controller", .flips_y_axis = false }, // from xone driver
   [xbox_one_s_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_S_USB_PRODUCT, .device_name = "Xbox Wireless Controller", .flips_y_axis = false },
   //[xbox_one_s_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_S_BLUETOOTH_PRODUCT, .device_name = "Xbox Wireless Controller", .flips_y_axis = false },
   //[xbox_one_s_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_S_BLUETOOTH_2_PRODUCT, .device_name = "Xbox Wireless Controller", .flips_y_axis = false },
   [xobx_one_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_PRODUCT, .device_name = "Xbox One Controller", .flips_y_axis = false },
   [xbox_one_2_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_2_PRODUCT, .device_name = "Xbox One Controller", .flips_y_axis = false },
   [xbox_one_elite_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_ELITE_PRODUCT, .device_name = "Xbox One Elite Controller", .flips_y_axis = false },
   //[xbox_one_elite_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_ELITE_2_PRODUCT, .device_name = "Xbox One Elite Controller", .flips_y_axis = false },
   [xobx_one_elite_series_2_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_ELITE_SERIES_2_USB_PRODUCT, .device_name = "Xbox One Elite Controller", .flips_y_axis = false },
   [xobx_one_elite_series_2_2_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_ELITE_SERIES_2_PRODUCT, .device_name = "Xbox One Elite Controller", .flips_y_axis = false },
   //[xbox_one_elite_series_2_index] = { .vendor_id = USB_MICROSOFT_VENDOR, .product_id = USB_XBOX_ONE_ELITE_SERIES_2_2_PRODUCT, .device_name = "Xbox One Elite Controller", .flips_y_axis = false },
};

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

struct AxisScale {
   int end_value;
   int start_value;
   bool uses_start_value;
} *axis_scales[ABS_CNT] = {NULL, NULL, NULL, NULL, NULL, NULL};

static void init_AxisTransform()
{
   for (int i=0; i<ABS_CNT; i++)
      axis_scales[i] = NULL;
}

static void parse_AxisScale(struct AxisScale *this, const char descriptor_[])
{
   char *descriptor = strdup(descriptor_);

   int split_index = strcspn(descriptor, ":");
   char *end_descriptor = &descriptor[split_index];
   if ( (this->uses_start_value=  *end_descriptor != '\0') )
   {
      *end_descriptor++ = '\0';
      this->start_value = (int)strtol(descriptor, NULL, 0);
   }
   else
      end_descriptor = descriptor;

   this->end_value = (int)strtol(end_descriptor, NULL, 0);

   free(descriptor);
}

static void add_AxisScale(int axis_code, const char descriptor[])
{
   if (axis_code < 0) return;
   struct AxisScale *axis_scale = NULL;

   int whitespace_end = strspn(descriptor, " \n\r\t\f\v");
   if (descriptor[whitespace_end] != '\0')
   {
      axis_scale = malloc(sizeof(struct AxisScale));
      if (axis_scale == NULL) { fprintf(stderr, "out of memory"); exit(-ENOMEM); }

      parse_AxisScale(axis_scale, descriptor);
   }

   if (axis_scales[axis_code] != NULL)
      free(axis_scales[axis_code]);

   axis_scales[axis_code] = axis_scale;
}

#define AxisName_none_index 7
const struct AxisName {
   const char *name;
   int code;
} sorted_axis_names[] = {
   {"brake", ABS_BRAKE},
   {"dpadx", ABS_HAT0X},
   {"dpady", ABS_HAT0Y},
   {"gas", ABS_GAS},
   {"lx", ABS_X},
   {"ly", ABS_Y},
   {"lz", ABS_Z},
   {"none", -1},
   {"rudder", ABS_RUDDER},
   {"rx", ABS_RX},
   {"ry", ABS_RY},
   {"rz", ABS_RZ},
   {"throttle", ABS_THROTTLE},
   {"wheel", ABS_WHEEL},
   {"x", ABS_X},
   {"y", ABS_Y},
   {"z", ABS_Z},
};

// length > 0, finds the index of the name or the next bigger name
static int search_axis_name(const char test_name[], int start_index, size_t length)
{
   int name_index = start_index + length/2;
   struct AxisName name = sorted_axis_names[name_index];

   int comparison = strcmp(test_name, name.name);
   if (comparison == 0)
      return name_index;

   if (comparison < 0)
   {
      length = name_index - start_index;
      if (length < 1)
         return name_index;

      return search_axis_name(test_name, start_index, length);
   }

   length += start_index;
   start_index = name_index+1;
   length -= start_index;
   if (length < 1)
      return name_index+1;

   return search_axis_name(test_name, start_index, length);
}

static struct AxisName parse_axis_name(const char descriptor[], const char **remainder)
{
   if (descriptor[0] == '\0') {
      *remainder = descriptor;
      return sorted_axis_names[AxisName_none_index];
   }

   char *compressed = strdup(descriptor);

   // remove non-alphanumeric characters
   int src_i=0, dest_i=0;
   char current_character;
   while ( (current_character=  descriptor[src_i]) != '\0')
   {
      if (isalnum(current_character))
      {
         compressed[dest_i++] = current_character | 0x20;  // to lower case
      }
      else if (!isspace(current_character) && current_character != '-' && current_character != '_')
      {
         break;
      }
      src_i++;
   }
   if (remainder != NULL) *remainder = &descriptor[src_i];
   compressed[dest_i] = '\0';

   int names_count = sizeof(sorted_axis_names) / sizeof(sorted_axis_names[0]);
   int i = search_axis_name(compressed, 0, names_count);

   free(compressed);

   return (i < names_count)? sorted_axis_names[i] : sorted_axis_names[AxisName_none_index];
}

static void set_axes_scales(const char scales_string[])
{
   if (scales_string[0] == '\0') return;

   char *key_value_pairs = strdup(scales_string);
   for (char *key_value_string = strtok(key_value_pairs, ","); key_value_string != NULL; key_value_string = strtok(NULL, ","))
   {
      int delimiter_index = strcspn(key_value_string, "=");
      if (key_value_string[delimiter_index] == '\0')
      {
         fprintf(stderr, "argument error: invalid argument \"%s\" given to --axes-scale\n", key_value_string);
         continue;
      }

      key_value_string[delimiter_index] = '\0';
      char *key_string = key_value_string;
      char *value_string = &key_value_string[delimiter_index+1];

      int axis_code = parse_axis_name(key_string, NULL).code;
      add_AxisScale(axis_code, value_string);
   }

   free(key_value_pairs);
}


static struct uinput_user_dev default_udev_settings = {
   .absmin[ABS_X]  = 35, .absmax[ABS_X]  = 218, .absfuzz[ABS_X]  = 1, .absflat[ABS_X] = 0,
   .absmin[ABS_Y]  = 35, .absmax[ABS_Y]  = 218, .absfuzz[ABS_Y]  = 1, .absflat[ABS_Y] = 0, // flippying the Y axis will change its range ends slightly
   .absmin[ABS_RX] = 43, .absmax[ABS_RX] = 215, .absfuzz[ABS_RX] = 1, .absflat[ABS_RX] = 0,
   .absmin[ABS_RY] = 43, .absmax[ABS_RY] = 215, .absfuzz[ABS_RY] = 1, .absflat[ABS_RY] = 0,
   .absmin[ABS_HAT0X] = 43, .absmax[ABS_HAT0X] = 215, .absfuzz[ABS_HAT0X] = 1, .absflat[ABS_HAT0X] = 0,
   .absmin[ABS_HAT0Y] = 43, .absmax[ABS_HAT0Y] = 215, .absfuzz[ABS_HAT0Y] = 1, .absflat[ABS_HAT0Y] = 0,
   .absmin[ABS_Z]  = 40, .absmax[ABS_Z]  = 190, .absfuzz[ABS_Z]  = 4, .absflat[ABS_Z] = 0,
   .absmin[ABS_RZ] = 40, .absmax[ABS_RZ] = 190, .absfuzz[ABS_RZ] = 4, .absflat[ABS_RZ] = 0,
   .absmin[ABS_THROTTLE]  = 43, .absmax[ABS_THROTTLE]  = 215, .absfuzz[ABS_THROTTLE]  = 4, .absflat[ABS_THROTTLE] = 0, // if used with triggers, 210 or more means fully depressed
   .absmin[ABS_RUDDER] = 43, .absmax[ABS_RUDDER] = 215, .absfuzz[ABS_RUDDER] = 4, .absflat[ABS_RUDDER] = 0,
   .absmin[ABS_WHEEL]  = 35, .absmax[ABS_WHEEL]  = 218, .absfuzz[ABS_WHEEL]  = 1, .absflat[ABS_WHEEL] = 0,
   .absmin[ABS_BRAKE] = 35, .absmax[ABS_BRAKE] = 218, .absfuzz[ABS_BRAKE] = 4, .absflat[ABS_BRAKE] = 0,
   .absmin[ABS_GAS] = 35, .absmax[ABS_GAS] = 218, .absfuzz[ABS_GAS] = 4, .absflat[ABS_GAS] = 0,
};
static struct uinput_user_dev uinput_dev;

static void flip_axis_bounds(struct uinput_user_dev* udev, int axis_code)
{
   if (axis_code < 0) return;

   int old_max = udev->absmax[axis_code];
   udev->absmax[axis_code] = udev->absmin[axis_code] ^ 0xff;
   udev->absmin[axis_code] = old_max ^ 0xff;
}
static void use_axis_normally(int axis_index)
{
   if (axis_index < 2)
      uses_thumbstick_left = thumbstick_normal;
   else if (axis_index < 4)
      uses_thumbstick_right = thumbstick_normal;
   else if (axis_index == 4)
      uses_trigger_left = trigger_normal;
   else
      uses_trigger_right = trigger_normal;
}
static void combine_axes(__attribute_maybe_unused__ struct uinput_user_dev *device_settings, int lower_axis, int upper_axis, int axis_index)
{
   if (lower_axis >= 0)
   {
      use_axis_normally(axis_index);
      if (axis_index >= 0)
         axis_code_values[axis_index].lo = lower_axis;
   }
   if (upper_axis >= 0)
   {
      use_axis_normally(axis_index);
      if (axis_index >= 0)
         axis_code_values[axis_index].hi = upper_axis;
   }
}
static void uncombine_axis(__attribute_maybe_unused__ struct uinput_user_dev *device_settings, int axis_code, int axis_index)
{
   if (axis_code < 0) return;
   
   if (axis_index >= 0)
   {
      use_axis_normally(axis_index);
      axis_code_values[axis_index].hi = axis_code;
      axis_code_values[axis_index].lo = -1;
   }
}
static void clear_axis(struct uinput_user_dev *device_settings, int axis_index)
{
   uncombine_axis(device_settings, axis_code_values[axis_index].lo, -1);
   uncombine_axis(device_settings, axis_code_values[axis_index].hi, -1);
   axis_code_values[axis_index] = (struct AxisCode){ -1, -1 };
}

static int get_axis_index(const char axis_string[])
{
   switch(axis_string[0] | 0x20)
   {
      case 'x': return thumbl_x_index;
      case 'y': return thumbl_y_index;
      case 'z': return trigger_l_index;
      case 'l':
         switch(axis_string[1] | 0x20)
         {
            case 'x': return thumbl_x_index;
            case 'y': return thumbl_y_index;
            case 'z':
            default: return trigger_l_index;
         }
      case 'r':
         switch(axis_string[1] | 0x20)
         {
            case 'x': return thumbr_x_index;
            case 'y': return thumbr_y_index;
            case 'z':
            default: return trigger_r_index;
         }
      default:
         fprintf(stderr, "argument error: unsupported analog input \"%s\" for --axes-map\n", axis_string);
         return -1;
   }
}

const char *axis_names[] = {
   "Thumb Left X",
   "Thumb Left Y",
   "Thumb Right X",
   "Thumb Right Y",
   "Trigger L",
   "Trigger R",
};
static void set_single_axis_map(int axis_index, char axis_name_expression[])
{
   const char *upper_axis_name;
   struct AxisName axis_name = parse_axis_name(axis_name_expression, &upper_axis_name);

   if (upper_axis_name[0] != '+')
   {
      fprintf(stdout, "map %s to %s\n", axis_names[axis_index], axis_name.name);
      uncombine_axis(&uinput_dev, axis_name.code, axis_index);
      return;
   }
   ((char*)upper_axis_name++)[0] = '\0';

   struct AxisName axis_name_hi = parse_axis_name(&upper_axis_name[0], NULL);

   fprintf(stdout, "map %s (low half) to %s, %s (high half) to %s\n", axis_names[axis_index], axis_name.name, axis_names[axis_index], axis_name_hi.name);
   combine_axes(&uinput_dev, axis_name.code, axis_name_hi.code, axis_index);
}

static void set_axes_map(const char mappings_string_[])
{
   if (mappings_string_[0] == '\0') return;

   char *mappings_string = strdup(mappings_string_);
   for (char *key_value_string = strtok(mappings_string, ","); key_value_string != NULL; key_value_string = strtok(NULL, ","))
   {
      int delimiter_index = strcspn(key_value_string, "=");
      if (key_value_string[delimiter_index] == '\0')
      {
         fprintf(stderr, "argument error: invalid argument \"%s\" was passed to --axes-map.\n", key_value_string);
         continue;
      }
      key_value_string[delimiter_index] = '\0';

      char *key_string = key_value_string;
      char *value_string = &key_value_string[delimiter_index+1];
      int axis_index = get_axis_index(key_string);
      set_single_axis_map(axis_index, value_string);
   }

   free(mappings_string);
}

static void set_raw_absinfo()
{
   uses_raw_mode = true;

   int axis_codes[] = {ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ, ABS_HAT0X, ABS_HAT0Y, ABS_THROTTLE, ABS_RUDDER, ABS_GAS, ABS_BRAKE, ABS_WHEEL};
   int axes_number = sizeof(axis_codes) / sizeof(axis_codes[0]);

   for (int i=0; i<axes_number; i++)
   {
      int axis_code = axis_codes[i];
      uinput_dev.absmin[axis_code]  = 0;  uinput_dev.absmax[axis_code]  = 255;
   }
}

/** Expects the command line settings to a comma separated list of assignments. The allowed variables are LX, LY, L, RX, RY, R and the allowed values are unsigned integer literals.
 *  The array_offset must be the offsetof(struct uinput_user_dev, …) of an array member field.
  */
static void set_axis_absinfo(ptrdiff_t array_offset, const char command_line_settings_[])
{
   if (command_line_settings_[0] == '\0') return;

   signed int *absinfo_array = (signed int*)((char*) &uinput_dev + array_offset);
   char *command_line_settings = strdup(command_line_settings_);

   for ( char *next_item = strtok(command_line_settings, ","); next_item != NULL; next_item = strtok(NULL, ",") )
   {
      int axis_name_length = strcspn(next_item, "=");
      if (next_item[axis_name_length] == '\0')
      {
         continue;
      }
      next_item[axis_name_length] = '\0';

      char *axis_value_string = &next_item[axis_name_length + 1];
      int axis_value = (int)strtoul(axis_value_string, NULL, 0);

      int axis_code = parse_axis_name(next_item, NULL).code;
      if (axis_code < 0)
      {
         continue;
      }

      absinfo_array[axis_code] = axis_value;
   }

   free(command_line_settings);
}

static bool uinput_create(int i, struct ports *port, unsigned char type)
{
   fprintf(stderr, "connecting on port %d\n", i);
   port->uinput = open(uinput_path, O_RDWR | O_NONBLOCK);

   // buttons
   ioctl(port->uinput, UI_SET_EVBIT, EV_KEY);
   int length = sizeof(button_code_values) / sizeof(button_code_values[0]);
   for (int j=0; j < length; j++)
   {
      int button_code = button_code_values[j];
      if (button_code != -1)
         ioctl(port->uinput, UI_SET_KEYBIT, button_code);
   }

   if (uses_trigger_left == trigger_binary)
      ioctl(port->uinput, UI_SET_KEYBIT, trigger_buttons[0]);
   if (uses_trigger_right == trigger_binary)
      ioctl(port->uinput, UI_SET_KEYBIT, trigger_buttons[1]);
   if (uses_thumbstick_left == thumbstick_dpad || uses_thumbstick_left == thumbstick_dpad_sensitive || uses_thumbstick_right == thumbstick_dpad || uses_thumbstick_right == thumbstick_dpad_sensitive)
   {
      int length = sizeof(dpad_button_codes) / sizeof(dpad_button_codes[0]);
      for (int j=0; j < length; j++)
      {
         ioctl(port->uinput, UI_SET_KEYBIT, dpad_button_codes[j]);
      }
   }

   // axis
   ioctl(port->uinput, UI_SET_EVBIT, EV_ABS);  // do we need to toggle this off when no axes are used?
   for (int i=0; i < AXIS_COUNT; i++)
   {
      int code = axis_code_values[i].lo;
      if (code >= 0)
         ioctl(port->uinput, UI_SET_ABSBIT, code);

      code = axis_code_values[i].hi;
      if (code >= 0)
         ioctl(port->uinput, UI_SET_ABSBIT, code);
   }

   // rumble
   ioctl(port->uinput, UI_SET_EVBIT, EV_FF);
   ioctl(port->uinput, UI_SET_FFBIT, FF_PERIODIC);
   ioctl(port->uinput, UI_SET_FFBIT, FF_SQUARE);
   ioctl(port->uinput, UI_SET_FFBIT, FF_TRIANGLE);
   ioctl(port->uinput, UI_SET_FFBIT, FF_SINE);
   ioctl(port->uinput, UI_SET_FFBIT, FF_RUMBLE);
   uinput_dev.ff_effects_max = MAX_FF_EVENTS;

   snprintf(uinput_dev.name, sizeof(uinput_dev.name), device_name, i+1);
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
      bool ignores_button = (uses_trigger_left == trigger_binary && button_code == trigger_buttons[0]) || (uses_trigger_right == trigger_binary && button_code == trigger_buttons[1]);
      if (!ignores_button)
      {
         int e_count = *events_count;
         events[e_count].type = EV_KEY;
         events[e_count].code = button_code;
         events[e_count].value = (single_button_pressed_mask != 0);
         e_count++;
         *events_count = e_count;
      }

      previous_button_state &= ~single_button_mask;
      previous_button_state |= single_button_pressed_mask;
      *result_button_state = previous_button_state;
   }
}


#define TAN_PI_8 0.414213562373095 // (sqrt(2.0) - 1.0)
#define TAN_3PI_8 (1.0 / TAN_PI_8)
#define HORIZONTAL_THRESHOLD (int)(250 * TAN_3PI_8)

static bool is_dpad_pressed(int axis_value, int perpendicular_value, int min_axis_value)
{
   if (axis_value < 0)
      axis_value = -axis_value;
   if (axis_value < min_axis_value || axis_value == 0)
      return false;

   int slope = perpendicular_value * 250 / axis_value;
   if (slope < 0)
      slope = -slope;
   return slope <= HORIZONTAL_THRESHOLD;
}

#define DPAD_FILTER_LENGTH 4  // 2 * filter length -1 = number of available duty cycles
static struct DeltaModulator {
   unsigned char unit_duration;  // time duration of a unit of equal return values
   signed char duty_cycle_units; // keydown to keyup ratio = 1:-n or +n:1, 1 complement, negative values represent duty cycles of keyup
   unsigned char time;
} thumbstick_filter[AXIS_COUNT] = {
   { .unit_duration = 4, .duty_cycle_units = 0, .time = 0, },
   { .unit_duration = 4, .duty_cycle_units = 0, .time = 0, },
   { .unit_duration = 4, .duty_cycle_units = 0, .time = 0, },
   { .unit_duration = 4, .duty_cycle_units = 0, .time = 0, },
};
int step_levels[] = {
   15 * 15, // 0
   37 * 37, // 1/4
   50 * 50, // 1/3
   64 * 64, // 1/2
   75 * 75, // 2/3
   87 * 87, // 3/4
   99 * 99, // 1
};
static signed char get_duty_cycle(int percent_squared)
{
   if (percent_squared <= step_levels[0]) return 0;

   if (percent_squared > step_levels[4])
   {
      if (percent_squared > step_levels[5])
         return ~0;
      
      return 3;
   }

   if (percent_squared <= step_levels[2])
   {
      if (percent_squared <= step_levels[1])
         return ~3;
      else
         return ~2;
   }
   else
   {
      if (percent_squared <= step_levels[3])
         return 1;
      else
         return 2;
   }
}

static void update_thumbstick_filter(struct DeltaModulator *filter, int percent_squared)
{
   signed char chosen_duty_cycle = get_duty_cycle(percent_squared);
   filter->duty_cycle_units = chosen_duty_cycle;

   unsigned unit_duration = filter->unit_duration;
   unsigned reset_time = ((chosen_duty_cycle < 0 ? ~chosen_duty_cycle : chosen_duty_cycle) + 1) * unit_duration;
   if (filter->time >= reset_time)
      filter->time = 0;
}

static bool read_thumbstick_filter(struct DeltaModulator *filter)
{
   bool is_inversed = filter->duty_cycle_units < 0;
   unsigned duty_cycle_units = (is_inversed? 1 : filter->duty_cycle_units);
   unsigned threshold = duty_cycle_units * filter->unit_duration;
   bool bit_value = filter->time < threshold;

   filter->time++;

   //if (filter == &thumbstick_filter[2])
   //{
   //   putc(bit_value ? '*' : ' ', stderr);
   //}

   return bit_value;
}

#define axis_value_to_signed(axis_value) ((int)(axis_value) + SCHAR_MIN)

static int signed_to_axis_value(int signed_value, int axis_index, enum AxisDivision axis_division)
{
   if (axis_division == full_axis)
      return signed_value - SCHAR_MIN;

   int start_value, end_value;
   if (axis_division == upper_half_axis)
   {
      start_value = axis_natural_ranges[axis_index].min;
      end_value = axis_natural_ranges[axis_index].max;
   }
   else
   {
      end_value = axis_natural_ranges[axis_index].min ^ 0xff;
      start_value = axis_natural_ranges[axis_index].max ^ 0xff;
      signed_value = -signed_value;
   }

   int signed_end_value = axis_value_to_signed(end_value);
   int axis_value = (int)(signed_value * ((float)(end_value - start_value) / (float)signed_end_value));
   return axis_value >= 0 ? axis_value + start_value : start_value;
}

static bool approx_deltamodulation(struct DeltaModulator *filter, int axis_value, int current_axis)
{
   int max_length = axis_value_to_signed(uinput_dev.absmax[current_axis]);
   int max_length_squared = max_length * max_length;
   int tilt_length_squared = axis_value * axis_value;
   int percent_squared = tilt_length_squared * 10000 / max_length_squared;

   update_thumbstick_filter(filter, percent_squared);

   return read_thumbstick_filter(filter);
}

static void map_thumbstick_to_dpad(struct input_event events[], int *events_count, struct ports *port, int current_axis, unsigned char payload[], int axis_index, enum ThumbstickMode thumbstick_mode)
{
   //righthand 2D coordinates
   signed char axis_value = axis_value_to_signed(payload[axis_index]);
   signed char perpendicular_axis_value = axis_value_to_signed(payload[axis_index ^ 1]);
   bool uses_axis = is_dpad_pressed(axis_value, perpendicular_axis_value, 20);

   if (thumbstick_mode == thumbstick_dpad_sensitive)
   {
      struct DeltaModulator *filter = &thumbstick_filter[axis_index];
      if (uses_axis)
         uses_axis = approx_deltamodulation(filter, axis_value, current_axis);
      else
         filter->time = 0;
   }

   int e_count = *events_count;
   bool is_vertical_axis = axis_index & 1;
   bool is_positive_axis = axis_value >= 0;
   int button_index = 2 * is_vertical_axis + is_positive_axis;

   int value = (is_positive_axis << 1) | 1;
   int opposite_value = value ^ 2;
   // turn off opposite direction
   if (port->axis[axis_index] == opposite_value)
   {
      events[e_count].type = EV_KEY;
      events[e_count].code = dpad_button_codes[button_index ^ 1];
      events[e_count].value = 0;
      e_count++;
   }

   if (uses_axis != (port->axis[axis_index] == value))
   {
      events[e_count].type = EV_KEY;
      events[e_count].code = dpad_button_codes[button_index];
      events[e_count].value = uses_axis;
      e_count++;
   }

   port->axis[axis_index] = value & (-2 | (int)uses_axis);
   *events_count = e_count;
}

static void add_axis_value(struct input_event events[], int *events_count, int axis_code, int new_value, uint8_t *old_value)
{
   struct input_event *event = &events[*events_count];

   int min = uinput_dev.absmin[axis_code];
   int max = uinput_dev.absmax[axis_code];
   if (new_value < min)
      new_value = min;
   else if (new_value > max)
      new_value = max;

   if (*old_value == new_value)
      return;

   *events_count += 1;
   event->type = EV_ABS;
   event->code = axis_code;
   *old_value = new_value;

   struct AxisScale *axis_scale = axis_scales[axis_code];
   if (axis_scale == NULL)
   {
      event->value = new_value;
      return;
   }

   float parameter = (float)(new_value - min) / (float)(max - min);

   int start_value = (axis_scale->uses_start_value) ? axis_scale->start_value : 0;

   int offset = (int)(parameter * (axis_scale->end_value - start_value));
   event->value = start_value + offset;
}

static void add_axis_event(struct input_event events[], int *events_count, unsigned char payload[], struct ports *port, int axis_index, int current_axis, enum AxisDivision axis_division)
{
   if (current_axis < 0) return;

   unsigned char value = payload[axis_index];

   bool is_left_shoulder_pressed_down = port->buttons & (1 << l_button_index);
   bool is_right_shoulder_pressed_down = port->buttons & (1 << r_button_index);

   if (axis_index == thumbl_y_index || axis_index == thumbr_y_index)
   {
      if (flips_y_axis)
      {
         if (axis_division == full_axis)
            value ^= 0xFF;
         else
            value ^= 0x7F;
            //axis_division = -axis_division;  // equivalent to value ^= 0xFF;
      }
   }

   value = signed_to_axis_value(axis_value_to_signed(value), axis_index, axis_division);

   if ((axis_index == trigger_l_index && uses_trigger_left == trigger_binary) || (axis_index == trigger_r_index && uses_trigger_right == trigger_binary))
   {
      value = value > uinput_dev.absmin[current_axis] + 10;
      if (uses_shoulder_button == shoulder_button_nand)
      {
         if (axis_index == trigger_l_index)
            value = value & !is_left_shoulder_pressed_down;
         else if (axis_index == trigger_r_index)
            value = value & !is_right_shoulder_pressed_down;
      }

      if (port->axis[axis_index] != value)
      {
         int e_count = *events_count;
         events[e_count].type = EV_KEY;
         events[e_count].code = (axis_index == trigger_l_index) ? trigger_buttons[0] : trigger_buttons[1];
         events[e_count].value = value;
         e_count++;
         *events_count = e_count;
         port->axis[axis_index] = value;
      }
      return;
   }
   else if (uses_shoulder_button == shoulder_button_nand)
   {
      if (is_left_shoulder_pressed_down && axis_index == trigger_l_index)
         value = uinput_dev.absmin[current_axis];
      else if (is_right_shoulder_pressed_down && axis_index == trigger_r_index)
         value = uinput_dev.absmin[current_axis];
   }

   if (uses_thumbstick_left != thumbstick_normal && (axis_index == thumbl_x_index || axis_index == thumbl_y_index))
   {
      map_thumbstick_to_dpad(events, events_count, port, current_axis, payload, axis_index, uses_thumbstick_left);
      return;
   }
   if (uses_thumbstick_right != thumbstick_normal && (axis_index == thumbr_x_index || axis_index == thumbr_y_index))
   {
      map_thumbstick_to_dpad(events, events_count, port, current_axis, payload, axis_index, uses_thumbstick_right);
      return;
   }

   add_axis_value(events, events_count, current_axis, value, &port->axis[axis_index]);
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
   
   for (int j = 0; j < BUTTON_COUNT; j++)
      add_button_event(events, &e_count, previous_buttons_state, &port->buttons, button_code_values, btns, j);

   for (int j = 0; j < AXIS_COUNT; j++)
   {
      int lower_axis = axis_code_values[j].lo;
      add_axis_event(events, &e_count, payload+3, port, j, axis_code_values[j].hi, lower_axis < 0? full_axis : upper_half_axis);
      add_axis_event(events, &e_count, payload+3, port, j, lower_axis, lower_half_axis);
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

   // thanks to https://github.com/dperelman/wii-u-gc-adapter
   if (uses_explicit_libusb_claim)
   {
      int tries_count = 0;
      while(libusb_claim_interface(a->handle, 0) != 0)
      {
         fprintf(stderr, "Error claiming interface 0 on adapter %#x from kernel, retry in 3 seconds\n", a->handle);
         sleep(3);
         fprintf(stderr, "\x1b[A(%d) ", ++tries_count); // thanks to https://stackoverflow.com/a/25103053
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

         if (uses_explicit_libusb_claim)
            libusb_release_interface(a->next->handle, 0);
         
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
   opt_device_name,
   opt_spoof_foreign,
   opt_claim,
   opt_implicit_use,
   opt_flip_y,
   opt_unflip_y,
   opt_use_z_thumbl,
   opt_use_z_thumbr,
   opt_use_z_bumpl,
   opt_use_z_bumpr,
   opt_use_z_select,
   opt_use_z,
   opt_use_abxyz_buttons,
   opt_use_literal_buttons,
   opt_use_foreign_buttons,
   opt_remap_dpad,
   opt_literal_dpad,
   opt_throttle_rudder,
   opt_brake_gas_wheel,
   opt_default_axes_map,
   opt_axes_map,
   opt_axes_scale,
   opt_thumb_left,
   opt_no_thumb_left,
   opt_dpad_left,
   opt_dpad_left_sensitive,
   opt_analog_dpad_left,
   opt_analog_dpad_left_flipped,
   opt_thumb_right,
   opt_no_thumb_right,
   opt_dpad_right,
   opt_dpad_right_sensitive,
   opt_analog_dpad_right,
   opt_analog_dpad_right_flipped,
   opt_quit_interrupt,
   opt_continue_interrupt,
   opt_deadzone,
   opt_tolerance,
   opt_min,
   opt_max,
   opt_no_shoulder,
   opt_shoulder_nand_trigger,
   opt_shoulder_and_trigger,
   opt_binary_trigger,
   opt_analog_trigger,
   opt_no_trigger,
};

static struct option options[] = {
   { "help", no_argument, 0, 'h' },
   { "raw", no_argument, 0, 'r' },
   { "flip-y-axis", no_argument, 0, opt_flip_y },
   { "unflip-y-axis", no_argument, 0, opt_unflip_y },
   { "vendor", required_argument, 0, opt_vendor },
   { "product", required_argument, 0, opt_product },
   { "device-name", required_argument, 0, opt_device_name },
   { "spoof-foreign", required_argument, 0, opt_spoof_foreign },
   { "claim", no_argument, 0, opt_claim },
   { "implicit-use", no_argument, 0, opt_implicit_use },
   { "z-to-thumbl", no_argument, 0, opt_use_z_thumbl },
   { "z-to-thumbr", no_argument, 0, opt_use_z_thumbr },
   { "z-to-bumpl", no_argument, 0, opt_use_z_bumpl },
   { "z-to-bumpr", no_argument, 0, opt_use_z_bumpr },
   { "z-to-select", no_argument, 0, opt_use_z_select },
   { "z-to-z", no_argument, 0, opt_use_z },
   { "enable-abxyz", no_argument, 0, opt_use_abxyz_buttons },
   { "literal-layout", no_argument, 0, opt_use_literal_buttons },
   { "foreign-layout", no_argument, 0, opt_use_foreign_buttons },
   { "remap-dpad", no_argument, 0, opt_remap_dpad },
   { "literal-dpad", no_argument, 0, opt_literal_dpad },
   { "throttle-rudder", no_argument, 0, opt_throttle_rudder },
   { "brake-gas-wheel", no_argument, 0, opt_brake_gas_wheel },
   { "default-axes-map", no_argument, 0, opt_default_axes_map },
   { "axes-map", required_argument, 0, opt_axes_map },
   { "axes-scale", required_argument, 0, opt_axes_scale },
   { "thumbstick-left", no_argument, 0, opt_thumb_left },
   { "thumbstick-left-none", no_argument, 0, opt_no_thumb_left },
   { "dpad-left", no_argument, 0, opt_dpad_left },
   { "dpad-left-sensitive", no_argument, 0, opt_dpad_left_sensitive },
   { "analog-dpad-left", no_argument, 0, opt_analog_dpad_left },
   { "analog-dpad-left-flipped", no_argument, 0, opt_analog_dpad_left_flipped },
   { "thumbstick-right", no_argument, 0, opt_thumb_right },
   { "thumbstick-right-none", no_argument, 0, opt_no_thumb_right },
   { "dpad-right", no_argument, 0, opt_dpad_right },
   { "dpad-right-sensitive", no_argument, 0, opt_dpad_right_sensitive },
   { "analog-dpad-right", no_argument, 0, opt_analog_dpad_right },
   { "analog-dpad-right-flipped", no_argument, 0, opt_analog_dpad_right_flipped },
   { "continue-on-interrupt", no_argument, 0, opt_continue_interrupt },
   { "quit-on-interrupt", no_argument, 0, opt_quit_interrupt },
   { "deadzone", required_argument, 0, opt_deadzone },
   { "change-tolerance", required_argument, 0, opt_tolerance },
   { "min-value", required_argument, 0, opt_min },
   { "max-value", required_argument, 0, opt_max },
   { "shoulder-none", required_argument, 0, opt_no_shoulder },
   { "shoulder-nand-trigger", no_argument, 0, opt_shoulder_nand_trigger },
   { "shoulder-also-trigger", no_argument, 0, opt_shoulder_and_trigger },
   { "trigger-buttons", no_argument, 0, opt_binary_trigger },
   { "trigger-axes", no_argument, 0, opt_analog_trigger },
   { "trigger-none", no_argument, 0, opt_no_trigger },
   { 0, 0, 0, 0 },
};

void swap_z_button_with_dpad_button(int z_code)
{
   int remapped_button_count = sizeof(REMAPPED_DPAD_DEFAULTS) / sizeof(REMAPPED_DPAD_DEFAULTS[0]);
   int first_remapped_index = sizeof(button_code_values) / sizeof(button_code_values[0]) - remapped_button_count;

   for (int i=0;  i < remapped_button_count; i++)
   {
      if (REMAPPED_DPAD_DEFAULTS[i] == z_code)
      {
         button_code_values[first_remapped_index + i] = DEFAULT_Z_CODE;
         break;
      }
   }
}

void process_options()
{
   if (vendor_id == 0)
      vendor_id = device_data[controller_index].vendor_id;
   if (product_id == 0)
      product_id = device_data[controller_index].product_id;
   if (device_name == NULL)
      device_name = device_data[controller_index].device_name;
   
   fprintf(stderr, "vendor_id = %#06x\n", vendor_id);
   fprintf(stderr, "product_id = %#06x\n", product_id);

   if (uses_foreign_buttons)
      memcpy(button_code_values, BUTTON_XBOX_VALUES, sizeof(button_code_values));
   else
      memcpy(button_code_values, BUTTON_LITERAL_VALUES, sizeof(button_code_values));

   button_code_values[z_button_index] = z_code;

   if (uses_remapped_dpad)
   {
      memcpy(button_code_values + left_button_index, REMAPPED_DPAD_DEFAULTS, sizeof(REMAPPED_DPAD_DEFAULTS));

      if (z_code != DEFAULT_Z_CODE)
         swap_z_button_with_dpad_button(z_code);
   }

   if (uses_shoulder_button == shoulder_button_none)
   {
      button_code_values[l_button_index] = BTN_TL2;
      button_code_values[r_button_index] = BTN_TR2;
   }
   else
   {
      button_code_values[l_button_index] = BTN_TL;
      button_code_values[r_button_index] = BTN_TR;
   }

   if (uses_thumbstick_left != thumbstick_normal)
   {
      clear_axis(&uinput_dev, thumbl_x_index);
      clear_axis(&uinput_dev, thumbl_y_index);

      if (uses_thumbstick_left == thumbstick_analog_dpad)
      {
         uncombine_axis(&uinput_dev, ABS_HAT0X, thumbl_x_index);
         uncombine_axis(&uinput_dev, ABS_HAT0Y, thumbl_y_index);
      }
      else if (uses_thumbstick_left == thumbstick_analog_dpad_flipped)
      {
         uncombine_axis(&uinput_dev, ABS_HAT0Y, thumbl_x_index);
         uncombine_axis(&uinput_dev, ABS_HAT0X, thumbl_y_index);
      }
   }
   if (uses_thumbstick_right != thumbstick_normal)
   {
      clear_axis(&uinput_dev, thumbr_x_index);
      clear_axis(&uinput_dev, thumbr_y_index);

      if (uses_thumbstick_right == thumbstick_analog_dpad)
      {
         uncombine_axis(&uinput_dev, ABS_HAT0X, thumbr_x_index);
         uncombine_axis(&uinput_dev, ABS_HAT0Y, thumbr_y_index);
      }
      else if (uses_thumbstick_right == thumbstick_analog_dpad_flipped)
      {
         uncombine_axis(&uinput_dev, ABS_HAT0Y, thumbr_x_index);
         uncombine_axis(&uinput_dev, ABS_HAT0X, thumbr_y_index);
      }
   }
   if (uses_trigger_left != trigger_normal)
   {
      if (uses_trigger_left == trigger_binary)
      {
         axis_code_values[trigger_l_index] = (struct AxisCode){ -1, ABS_Z };
      }
      else
      {
         axis_code_values[trigger_l_index] = (struct AxisCode){ -1, -1 };
      }
   }
   if (uses_trigger_right != trigger_normal)
   {
      if (uses_trigger_right == trigger_binary)
      {
         axis_code_values[trigger_r_index] = (struct AxisCode){ -1, ABS_RZ };
      }
      else
      {
         axis_code_values[trigger_r_index] = (struct AxisCode){ -1, -1 };
      }
   }

   if (flips_y_axis)
   {
      struct AxisCode y_axis = axis_code_values[thumbl_y_index];
      flip_axis_bounds(&default_udev_settings, y_axis.lo);
      flip_axis_bounds(&default_udev_settings, y_axis.hi);
      struct AxisCode ry_axis = axis_code_values[thumbr_y_index];
      flip_axis_bounds(&default_udev_settings, ry_axis.lo);
      flip_axis_bounds(&default_udev_settings, ry_axis.hi);
   }
}

int main(int argc, char *argv[])
{
   struct udev *udev;
   struct udev_device *uinput;
   struct sigaction sa;
   uinput_dev = default_udev_settings;
   init_AxisTransform();

   memset(&sa, 0, sizeof(sa));

   while (1) {
      int option_index = 0;
      int c = getopt_long(argc, argv, "rh", options, &option_index);
      if (c == -1)
         break;

      if (c == 'h') {
         fprintf(stdout,
            "usage: wii-u-gc-adapter  [--help] [--vendor ⟨int⟩] [--product ⟨int⟩] [--device-name ⟨str⟩] [--fake-xbox ⟨int⟩] [⟨flag options as below⟩] \\\n"
            "                 [--axes-map [X=[⟨str⟩],][Y=[⟨str⟩],][RX=[⟨str⟩],][RY=[⟨str⟩],][L=[⟨str⟩],][R=[⟨str⟩],]\"\"] \\\n"
            "                 [--axes-scale [⟨str⟩=[⟨int⟩[:⟨int⟩]],]…\"\"]\n"
            "                 [--deadzone [⟨str⟩=⟨uint⟩,]…\"\"] \\\n"
            "                 [--change-tolerance [⟨str⟩=⟨uint⟩,]…\"\"] \\\n"
            "                 [--min-value [⟨str⟩=⟨uint⟩,]…\"\"] \\\n"
            "                 [--max-value [⟨str⟩=⟨uint⟩,]…\"\"] \\\n"
            "\n");
         fprintf(stdout,
            "--help, -h                 Display this help text.\n"
            "--raw                      (for testing) removes the adjustment of the input value range on analog input values, i.e. it sets min = 0, max = 255 instead of using the controller adjusted default range.\n"
            "--flip-y-axis              (default) reverses the received Y-axis value (for left thumbstick Y and right thumbstick Y) so that 0 produces 255 and 255 produces 0.\n"
            "                           Requires another Y axis inversion when used with xboxdrv. When an analog input is split into two axes, it flips each axis individually.\n"
            "--unflip-y-axis            leaves the Y axis signal value as it arrives (for ABS_Y and ABS_RY). Use this for games which expect genuine GameCube controller values.\n"
            "--continue-on-interrupt    (default) it tries to wait and retry when libusb interrupt occurs (for example when entering sleep).\n"
            "--quit-on-interrupt        will make the thread stop and exit when a libusb interrupt occurs. Mutually exclusive to \"--quit-on-interrupt\".\n"
            "--vendor and --product     correspond to the IDs associated to the event device that should be read. Default values are vendor = %#06x, product = %#06x.\n"
            "--device-name              allows users to provide a custom device name that replaces the \"Wii U Adapter…\" one.\n"
            "--spoof-foreign            allows users to immitate a false idenity with spoofed name, vendor and product ID. Unflips the y axis.\n"
            "                           Probably does not suffice. You are better off using xboxdrv's mimic-xbox configuration option together with \"--evdev\".\n"
            "                           values: 0 → no spoofing, 1 → Xbox 360, 1 → Xbox 360 Wireless, 3 → Xbox Wireless, 4 → Xbox S, 5 → Xbox One,\n"
            "                                   6 → Xbox One (2), 7 → Xbox One S, 8 → Xbox One Elite, 9 → Xbox One Elite Se. 2, 10 → Xbox One Elite Se. 2 (2)\n"
            "--claim                    turns on explicit USB claiming and releasing. Maybe prevents libusb ERRORs on startup. If claimed by other software, libusb errors will occur.\n"
            "--implicit-use             (default) turns off explicit USB claiming and releasing. It should still be working e.g. on recent Arch-based distros. Maybe problematic when started at system boot time.\n"
            "\n",
            USB_NINTENDO_VENDOR, USB_ID_PRODUCT
         );
         fprintf(stdout,
            "--z-to-thumbl              (default) activates a left thumbstick click (BTN_THUMBL) when pressing the Z button.\n"
            "                           This is useful for most PC games as they use BTN_THUMBL more often with gameplay relevance but almost never know BTN_Z.\n"
            "--z-to-thumbr              uses the right thumbstick click instead of the left one when pressing Z.\n"
            "--z-to-bumpl               uses the left shoulder button (LB) when pressing Z.\n"
            "--z-to-bumpr               uses the right shoulder button (RB) when pressing Z.\n"
            "--z-to-select              makes Z trigger the event for the XBOX controller \"back\" or \"select\" button, BTN_SELECT.\n"
            "--z-to-z                   triggers BTN_Z when pressing Z. PC games usually don't know this button.\n"
            "--literal-layout           (default) emits events BTN_A, BTN_B, BTN_X, BTN_Y when pressing A, B, X or Y.\n"
            "--enable-abxyz             Combines \"--literal-layout\" with \"--z-to-z\".\n"
            "--foreign-layout           emits events for BTN_SOUTH, BTN_WEST, BTN_EAST, BTN_NORTH instead of BTN_A, BTN_B, BTN_X, BTN_Y.\n"
            "                           This flag is mutually exclusive to \"--enable-abxyz\" and \"--literal-layout\". In Linux, BTN_A, BTN_B, BTN_X, BTN_Y are XBOX synonyms for cardinal direction names.\n"
            "                           Use this flag if you want the GCN buttons A, B, X, Y to be laid like on XBOX controllers. Possibly useful for playstation controller games.\n"
            "--remap-dpad               uses the D-pad instead to emit XBOX controller buttons which do not exist on GCN controllers. Nowdays, with control sticks, D-pad buttons lose relevance.\n"
            "                           The limitation of D-pad remapping is, you can only press two adjacent D-pad buttons at the same time but not more and not opposite ones.\n"
            "                           D-pad buttons react to the chosen flag \"--z-to-…\". It will swap the mappings of Z and the D-pad button whose default map is in conflict.\n"
            "                           If you still need the D-pad, you can either use the \"--dpad-right\" flag or use your keyboard in combination (WASD or arrow keys often replace the D-pad).\n"
            "                           default mapping: left → Xbox LB (BTN_TL), right → Xbox RB (BTN_TR), up → Xbox back/select (BTN_SELECT), down → right thumbstick click (BTN_THUMBR)\n"
            "--literal-dpad             (default) emits the proper D-pad button events (BTN_DPAD_UP, BTN_DPAD_LEFT …) when pressing on the D-pad. Mutually exclusive to \"--remap-dpad\".\n"
            "--trigger-buttons          makes the triggers (L and R) behave as binary buttons only (BTN_TL2 and BTN_TR2). Use \"--trigger-none\" to disable the analog triggers entirely.\n"
            "--trigger-axes             (default) uses the analog axes assigned to L and R when pressing L and R, by default these correspond to XBOX LT and RT.\n"
            "--trigger-none             deactivates analog L and R but recognizes the shoulder button events (fully depressed L or R).\n"
            "\n");
         fprintf(stdout,
            "--throttle-rudder          uses ABS_THROTTLE and ABS_RUDDER for ABS_RY and ABS_RX instead of the usual ABS_RX and ABS_RY. Flight or ship simulators might support these.\n"
            "                           Corresponds to \"--axis-map RY=throttle,RX=rudder\" and also unflips the Y axis.\n"
            "--brake-gas-wheel          uses ABS_WHEEL for ABS_X, ABS_GAS for upper ABS_Y and ABS_BRAKE for lower ABS_Y. Car simulators might support these\n"
            "                           Corresponds to \"--axis-map Y=brake+gas,X=wheel\" and also unflips the Y axis.\n"
            "--axes-map                 allows free mapping of the 6 analog dimensions of the controller, see command usage help above.\n"
            "                           value strings per axis: \"none\", \"x\", \"y\", \"z\", \"rx\", \"ry\", \"rz\", \"dpadx\", \"dpady\", \"brake\", \"gas\", \"wheel\", \"throttle\", \"rudder\"\n"
            "                           or a combination as pair \"⟨option 1⟩+⟨option 2⟩\". When combined, option 1 will receive inverted values and the center of the value range is where both axes split.\n"
            "                           You can also use \"lx\", \"ly\" and \"lz\" (instead of x, y, z) for compatibility with the previous version.\n"
            "                           A concatenation uses the first axis for the lower half of values, the 2nd axis for the upper half of values.\n"
            "--default-axes-map         resets the axes map to the default map \"X=x,Y=y,L=z,RX=rx,RY=ry,R=rz\".\n"
            "--axes-scale               permits to set the scale of any of the axis names (none, x, y, … see \"--axes-map\"). The string values for each axis consist of at least one int value (the end value)\n"
            "                           or two int values, separated with colon ':', which are the start and end value (start may be larger than end). If no start value is given, it is set to 0.\n"
            "                           Use the empty string as value to use the default scale and remove a custom scale.\n"
            "--thumbstick-left          (default) uses the left analog axes (ABS_X and ABS_Y) for the left thumbstick. This is the normal behaviour expected by games.\n"
            "--thumbstick-left-none     Deactivates the left thumbstick.\n"
            "--dpad-left                turns left thumbstick into a D-pad. Rarely ever useful I guess.\n"
            "--dpad-left-sensitive      like \"--dpad-left\" but uses a duty cycle (quickly presses key down and up) for each axis that corresponds to the tilting strength of the stick.\n"
            "                           The available duty cycles are: 0, 1/4, 1/3, 1/2, 2/3, 3/4, 1 and are triggered with increasing tilting strength. The D-pad button state changes in a multiple\n"
            "                           of 4 update frames. Games might compute acceleration which low-pass filters input so that high frequency button presses are ineffective.\n"
            "--analog-dpad-left         emits ABS_HAT0X and ABS_HAT0Y when using the left thumbstick.\n"
            "--analog-dpad-left-flipped emits ABS_HAT0Y and ABS_HAT0X instead of ABS_HAT0X and ABS_HAT0Y when using the left thumbstick.\n"
            "--thumbstick-right         (default) uses the right analog axes (ABS_RX and ABS_RY) for the right thumbstick. This is the normal behaviour expected by games.\n"
            "--thumbstick-right-none    Deactivates the right thumbstick.\n"
            "--dpad-right               turns right thumbstick into a D-pad. It is uncommon for a game to make use of right thumbstick and D-pad at the same time. Some RPGs however use the D-pad for menus.\n"
            "--dpad-right-sensitive     like \"--dpad-right\" but uses a duty cycle (quickly presses key down and up) that corresponds to the tilting strength of the stick. See \"--dpad-left-sensitive\".\n"
            "--analog-dpad-right        emits ABS_HAT0X and ABS_HAT0Y when using the right thumbstick.\n"
            "--analog-dpad-right-flipped emits ABS_HAT0Y and ABS_HAT0X instead of ABS_HAT0X and ABS_HAT0Y when using the right thumbstick.\n"
            "\n");
         fprintf(stdout,
            "--shoulder-none            (default) do NOT emit events for trigger buttons LT (BTN_TL2) and RT (BTN_TR2) when L or R are depressed fully, instead of LB (BTN_TL) and RB (BTN_TR).\n"
            "--shoulder-nand-trigger    ensures that LB/RB and LT/RT are never active together at the same time. LB/RB are activated when L and R are depressed fully.\n"
            "--shoulder-also-trigger    emits LB/RB while the trigger (LT/RT) is depressed fully. LB and RB only activate when LT/RT are active.\n"
            "\n"
            "--deadzone, --change-tolerance, --min-value and --max-value configure the analog axis event value.\n"
            "       \"Deadzone\" specifies a limit on the absolute value of the analog control element which suppresses events for smaller values, default value is '35' for L and R triggers.\n"
            "       \"Change Tolerance\" specifies the smallest value change of the analog value which suppresses events for smaller differences, default value is '1'\n"
            "       \"Min Value\" is the lowest analog value emitted from an analog axis.\n"
            "       \"Max Value\" is the maximum analog value emitted from an analog axis. If this is too high, then the maximum input value (required by some games) cannot be reached.\n"
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
         break;
      case opt_product:
         product_id = parse_id(optarg);
         break;
      case opt_device_name:
         if (device_name != NULL)
            free((char*)device_name);
         device_name = strdup(optarg);
         break;
      case opt_spoof_foreign:
         controller_index = (int)strtol(optarg, NULL, 0);
         if (controller_index >= no_controller_index)
            controller_index = gcn_adapter_index;
         if (device_name != NULL)
            free((char*)device_name);
         vendor_id = 0;
         product_id = 0;
         flips_y_axis = device_data[controller_index].flips_y_axis;
         break;
      case opt_continue_interrupt: quits_on_interrupt = false; break;
      case opt_quit_interrupt: quits_on_interrupt = true; break;
      case opt_claim: uses_explicit_libusb_claim = true; break;
      case opt_implicit_use: uses_explicit_libusb_claim = false; break;
      case opt_flip_y: flips_y_axis = true; break;
      case opt_unflip_y: flips_y_axis = false; break;

      case opt_use_z_thumbl: z_code = BTN_THUMBL; break;
      case opt_use_z_thumbr: z_code = BTN_THUMBR; break;
      case opt_use_z_bumpl: z_code = BTN_TL; break;
      case opt_use_z_bumpr: z_code = BTN_TR; break;
      case opt_use_z_select: z_code = BTN_SELECT; break;
      case opt_use_z: z_code = BTN_Z; break;
      case opt_use_abxyz_buttons: uses_foreign_buttons = false; z_code = BTN_Z; break;
      case opt_use_literal_buttons: uses_foreign_buttons = false; break;
      case opt_use_foreign_buttons: uses_foreign_buttons = true; break;
      case opt_remap_dpad: uses_remapped_dpad = true; break;
      case opt_literal_dpad: uses_remapped_dpad = false; break;

      case opt_axes_map: set_axes_map(optarg); break;
      case opt_axes_scale: set_axes_scales(optarg); break;
      case opt_throttle_rudder: set_axes_map("RY=throttle,RX=rudder"); flips_y_axis = false; break;
      case opt_brake_gas_wheel: set_axes_map("Y=brake+gas,X=wheel"); flips_y_axis = false; break;
      case opt_default_axes_map: set_axes_map("X=x,Y=y,L=z,RX=rx,RY=ry,R=rz"); break;
      case opt_thumb_left: uses_thumbstick_left = thumbstick_normal; break;
      case opt_no_thumb_left: uses_thumbstick_left = thumbstick_none; break;
      case opt_dpad_left: uses_thumbstick_left = thumbstick_dpad; break;
      case opt_dpad_left_sensitive: uses_thumbstick_left = thumbstick_dpad_sensitive; break;
      case opt_analog_dpad_left: uses_thumbstick_left = thumbstick_analog_dpad; break;
      case opt_analog_dpad_left_flipped: uses_thumbstick_left = thumbstick_analog_dpad_flipped; break;
      case opt_thumb_right: uses_thumbstick_right = thumbstick_normal; break;
      case opt_no_thumb_right: uses_thumbstick_right = thumbstick_none; break;
      case opt_dpad_right: uses_thumbstick_right = thumbstick_dpad; break;
      case opt_dpad_right_sensitive: uses_thumbstick_right = thumbstick_dpad_sensitive; break;
      case opt_analog_dpad_right: uses_thumbstick_right = thumbstick_analog_dpad; break;
      case opt_analog_dpad_right_flipped: uses_thumbstick_right = thumbstick_analog_dpad_flipped; break;

      case opt_no_shoulder: uses_shoulder_button = shoulder_button_none; break;
      case opt_shoulder_nand_trigger: uses_shoulder_button = shoulder_button_nand; break;
      case opt_shoulder_and_trigger: uses_shoulder_button = shoulder_button_and; break;
      
      case opt_binary_trigger: uses_trigger_left = uses_trigger_right = trigger_binary; break;
      case opt_analog_trigger: uses_trigger_left = uses_trigger_right = trigger_normal; break;
      case opt_no_trigger: uses_trigger_left = uses_trigger_right = trigger_none; break;

      case opt_deadzone: set_axis_absinfo(offsetof(struct uinput_user_dev, absflat), optarg); break;
      case opt_tolerance: set_axis_absinfo(offsetof(struct uinput_user_dev, absfuzz), optarg); break;
      case opt_min: set_axis_absinfo(offsetof(struct uinput_user_dev, absmin), optarg); break;
      case opt_max: set_axis_absinfo(offsetof(struct uinput_user_dev, absmax), optarg); break;
      }
   }

   process_options();

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
      if (desc.idVendor == USB_NINTENDO_VENDOR && desc.idProduct == USB_ID_PRODUCT)
         add_adapter(devices[i]);
   }

   if (count > 0)
      libusb_free_device_list(devices, 1);

   libusb_hotplug_callback_handle callback;

   int hotplug_capability = libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG);
   if (hotplug_capability) {
       int hotplug_ret = libusb_hotplug_register_callback(NULL,
             LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
             0, USB_NINTENDO_VENDOR, USB_ID_PRODUCT,
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
