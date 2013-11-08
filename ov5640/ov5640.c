/*
 * Driver for OV5640 CMOS Image Sensor from OmniVision
 *
 * Copyright (C) 2013, Alex Lei <alexvonduar@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ov5640.h"
#include "ov5640_regs.h"

#if defined(CONFIG_DBG)
#undef dev_dbg
#define dev_dbg(dev, format, arg...) dev_printk(KERN_DEBUG, dev, format, ##arg)
#endif

#define OV5640_DRIVER_NAME    "ov5640"

#define OV5640_GPIO_RESET 98
#define OV5640_GPIO_PWDN  167

#define OV5640_GPIO_PWDN_ON 1
#define OV5640_GPIO_PWDN_OFF 0

#define OV5640_BRIGHTNESS_MIN 0
#define OV5640_BRIGHTNESS_MAX 200
#define OV5640_BRIGHTNESS_STEP 100
#define OV5640_BRIGHTNESS_DEF 100

#define OV5640_CONTRAST_MIN 0
#define OV5640_CONTRAST_MAX 200
#define OV5640_CONTRAST_STEP 100
#define OV5640_CONTRAST_DEF 100

enum ov5640_model {
    OV5640_MODEL_COLOR,
    OV5640_MODEL_MONOCHROME,
};

enum ov5640_frame_size {
    OV5640_SIZE_QQVGA,
    OV5640_SIZE_QVGA,
    OV5640_SIZE_352x288,
    OV5640_SIZE_VGA,
    OV5640_SIZE_XGA,
    OV5640_SIZE_WXGA,
    OV5640_SIZE_UXGA,
    OV5640_SIZE_1080P,
    OV5640_SIZE_QXGA,
    OV5640_SIZE_FULL,
    OV5640_SIZE_LAST
};

static const struct v4l2_frmsize_discrete ov5640_frame_sizes[OV5640_SIZE_LAST] = {
    {160,120},
    {320,240},
    {352,288},
    {640,480},
    {1024,768},
    {1280,720},
    {1600,1200},
    {1920,1080},
    {2048,1536},
    {2592,1944}
};

typedef struct {
    enum v4l2_mbus_pixelcode code;
    enum v4l2_colorspace colorspace;
} ov5640_dataformat;

static const ov5640_dataformat ov5640_color_formats[] = {
    {V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
};

struct ov5640 {
    struct v4l2_subdev subdev;
    struct media_pad pad;
    struct v4l2_rect crop;  /* Sensor window */
    struct v4l2_rect curr_crop;
    struct v4l2_mbus_framefmt format;
    struct ov5640_platform_data *pdata;
    struct mutex power_lock; /* lock to protect power_count */
    int power_count;

    enum ov5640_model model;
    //struct aptina_pll pll;
    int reset;
    int pwdn;
    int res_index;

    struct v4l2_ctrl_handler ctrls;
    struct v4l2_ctrl *blc_auto;
    struct v4l2_ctrl *blc_offset;
};

#if 0
static struct ov5640 *to_ov5640(struct v4l2_subdev *sd)
{
    return container_of(sd, struct ov5640, subdev);
}
#else
#define to_ov5640(sd) container_of((sd), struct ov5640, subdev)
#endif

/* Find a data format by a pixel code in an array */
static const ov5640_dataformat
            *ov5640_find_dataformat(enum v4l2_mbus_pixelcode code)
{
    int i;

    dev_dbg(NULL, "[OV5640] %s: find format %08x\n", __func__, code);

    for (i = 0; i < ARRAY_SIZE(ov5640_color_formats); i++)
        if (ov5640_color_formats[i].code == code)
            return ov5640_color_formats + i;

    return NULL;
}

static int ov5640_read(struct i2c_client *client, u16 reg, u8 * val)
{
    int ret;
    /* We have 16-bit i2c addresses - care for endianess */
    unsigned char data[2] = { reg >> 8, reg & 0xff };

    ret = i2c_master_send(client, data, 2);
    if (ret < 2) {
        dev_err(&client->dev, "[OV5640] %s: i2c read error, reg: %x\n",
            __func__, reg);
        return ret < 0 ? ret : -EIO;
    }

    ret = i2c_master_recv(client, val, 1);
    if (ret < 1) {
        dev_err(&client->dev, "[OV5640] %s: i2c read error, reg: %x\n",
                __func__, reg);
        return ret < 0 ? ret : -EIO;
    }

    return 0;
}

static int ov5640_write(struct i2c_client *client, u16 reg, u8 val)
{
    int ret;
    unsigned char data[3] = { reg >> 8, reg & 0xff, val };

    ret = i2c_master_send(client, data, 3);
    if (ret < 3) {
        dev_err(&client->dev, "[OV5640] %s: i2c write error, reg: %x\n",
            __func__, reg);
        return ret < 0 ? ret : -EIO;
    }

    return 0;
}

#if 0
/*
 * convenience function to write 16 bit register values that are split up
 * into two consecutive high and low parts
 */
static int ov5640_write16(struct i2c_client *client, u16 reg, u16 val16)
{
    int ret;

    ret = ov5640_write(client, reg, val16 >> 8);
    if (ret)
        return ret;
    return ov5640_write(client, reg + 1, val16 & 0x00ff);
}
#endif

static int ov5640_write_array(struct i2c_client *client, struct ov5640_reg *vals)
{
    while (vals->addr != 0xffff || vals->val != 0xff) {
        int ret = ov5640_write(client, vals->addr, vals->val);
        if (ret < 0)
            return ret;
        vals++;
    }

    dev_dbg(&client->dev, "[OV5640] %s: Register list loaded\n", __func__);

    return 0;
}

static int ov5640_power_on(struct ov5640 * info)
{
    dev_dbg(NULL, "[OV5640] %s: power on\n", __func__);
    return 0;
}

static int ov5640_power_off(struct ov5640 * info)
{
    dev_dbg(NULL, "[OV5640] %s: power off\n", __func__);
    return 0;
}

#if 0
static int ov5640_sw_reset(struct ov5640 * info)
{
    return 0;
}
#endif

static int ov5640_hw_reset(struct ov5640 * info)
{
    dev_dbg(NULL, "[OV5640] %s: hardware reset\n", __func__);
    return 0;
}

/**
 * ov5640_find_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static int __ov5640_find_size(struct v4l2_frmsize_discrete * requestedsize)
{
    int i = 0;
    u32 requested_pixels = requestedsize->width * requestedsize->height;
    dev_dbg(NULL, "[OV5640] %s: find a supported frame size\n", __func__);

    for (i = 0; i < OV5640_SIZE_LAST; i++) {
        if (ov5640_frame_sizes[i].height * ov5640_frame_sizes[i].width >= requested_pixels)
        return i;
    }

    /* couldn't find a match, return the max size as a default */
    return OV5640_SIZE_FULL;
}

/**
 * ov5640_v4l2_try_fmt_cap - Find the best match for a requested image capture size
 * @ov5640_frame_size: a ointer to the structure which specifies requested image size
 * 
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static int ov5640_v4l2_try_fmt_cap(struct v4l2_frmsize_discrete * requestedsize) 
{
    int isize;

    dev_dbg(NULL, "[OV5640] %s: requested %dx%d\n", __func__, requestedsize->width, requestedsize->height);

    isize = __ov5640_find_size(requestedsize);

    requestedsize->width = ov5640_frame_sizes[isize].width;
    requestedsize->height = ov5640_frame_sizes[isize].height;

    return isize;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */

static int ov5640_s_ctrl(struct v4l2_ctrl * ctrl)
{
    dev_dbg(NULL, "[OV5640] %s: set control\n", __func__);

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE_AUTO:
    default:
        break;
    }
    return 0;
}

static struct v4l2_ctrl_ops ov5640_ctrl_ops = {
    .s_ctrl = ov5640_s_ctrl,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

static int ov5640_g_chip_ident(struct v4l2_subdev * sd, struct v4l2_dbg_chip_ident *chip)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);
    u8 val;

    dev_dbg(&i2c->dev, "[OV5640] %s: get chip ident\n", __func__);

    return ov5640_read(i2c, 0x3000, &val);
}

// NULL static int ov5640_log_status(struct v4l2_subdev *sd);
// NULL static int ov5640_s_io_pin_config(struct v4l2_subdev *sd, size_t n, struct v4l2_subdev_io_pin_config *pincfg);
// NULL static int ov5640_init(struct v4l2_subdev *sd, u32 val);
// NULL static int ov5640_load_fw(struct v4l2_subdev *sd);
// NULL static int ov5640_reset(struct v4l2_subdev *sd, u32 val);
// NULL static int ov5640_s_gpio(struct v4l2_subdev *sd, u32 val);

/* supported controls */
static struct v4l2_ctrl_config ov5640_ctrls[] = {
    {
        .ops = &ov5640_ctrl_ops,
        .id = V4L2_CID_GAIN,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Gain",
        .min = 0,
        .max = (1 << 12) - 1 - 0x0020,
        .step = 1,
        .def = 0x0020,
        .flags = 0,
    }, {
        .ops = &ov5640_ctrl_ops,
        .id = V4L2_CID_EXPOSURE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Exposure",
        .min = 0,
        .max = 2047,
        .step = 1,
        .def = 0x01fc,
        .flags = 0,
    }, {
        .ops = &ov5640_ctrl_ops,
        .id = V4L2_CID_RED_BALANCE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Red Balance",
        .min = -1 << 9,
        .max = (1 << 9) - 1,
        .step = 1,
        .def = 0,
        .flags = 0,
    }, {
        .ops = &ov5640_ctrl_ops,
        .id = V4L2_CID_BLUE_BALANCE,
        .type = V4L2_CTRL_TYPE_INTEGER,
        .name = "Blue Balance",
        .min = -1 << 9,
        .max = (1 << 9) - 1,
        .step = 1,
        .def = 0,
        .flags = 0,
    }, {
        .ops = &ov5640_ctrl_ops,
        .id      = V4L2_CID_HFLIP,
        .type    = V4L2_CTRL_TYPE_BOOLEAN,
        .name    = "Mirror",
        .min = 0,
        .max = 1,
        .step    = 1,
        .def = 0,
        .flags = 0,
    }, {
        .ops = &ov5640_ctrl_ops,
        .id      = V4L2_CID_VFLIP,
        .type    = V4L2_CTRL_TYPE_BOOLEAN,
        .name    = "Vflip",
        .min = 0,
        .max = 1,
        .step    = 1,
        .def = 0,
        .flags = 0,
    }, {
    }
};

static void __copy_ctrl_config_query(struct v4l2_queryctrl * qc, struct v4l2_ctrl_config * cc)
{
    qc->id = cc->id;
    qc->type = cc->type;
    memcpy(qc->name, cc->name, 32);
    qc->minimum = cc->min;
    qc->maximum = cc->max;
    qc->step = cc->step;
    qc->default_value = cc->def;
    qc->flags = cc->flags;
}

static int ov5640_queryctrl(struct v4l2_subdev * sd, struct v4l2_queryctrl * qc)
{
    int i;
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: queryctrl called\n", __func__);

    for (i = 0; i < ARRAY_SIZE(ov5640_ctrls); i++)
        if (qc->id && qc->id == ov5640_ctrls[i].id) {
            __copy_ctrl_config_query(qc, &(ov5640_ctrls[i]));
            return 0;
        }

    return -EINVAL;
}

// NULL static int ov5640_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
// NULL static int ov5640_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
// NULL static int ov5640_g_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
// NULL static int ov5640_s_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
// NULL static int (*try_ext_ctrls)(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls);
// NULL static int (*querymenu)(struct v4l2_subdev *sd, struct v4l2_querymenu *qm);
// NULL static int (*g_std)(struct v4l2_subdev *sd, v4l2_std_id *norm);
// NULL static int (*s_std)(struct v4l2_subdev *sd, v4l2_std_id norm);
// NULL static long (*ioctl)(struct v4l2_subdev *sd, unsigned int cmd, void *arg);

#ifdef CONFIG_VIDEO_ADV_DEBUG

static int ov5640_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);
    int ret;
    u8 val;
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: get register\n", __func__);

    if (reg->reg & ~0xffff)
        return -EINVAL;

    reg->size = 1;

    ret = ov5640_read(i2c, reg->reg, &val);
    if (!ret)
        reg->val = (__u64)val;

    return ret;
}

static int ov5640_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: set register\n", __func__);

    if (reg->reg & ~0xffff || reg->val & ~0xff)
        return -EINVAL;

    return ov5640_write(i2c, reg->reg, reg->val);
}

#endif

static int ov5640_s_power(struct v4l2_subdev *sd, int on)
{
    int ret = 0;
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: set power %d\n", __func__, on);

    if (on) {
        ret = ov5640_power_on(info);
        ov5640_hw_reset(info);
    }
    else {
        ret = ov5640_power_off(info);
    }
    return 0;
}

// NULL static int (*interrupt_service_routine)(struct v4l2_subdev *sd, u32 status, bool *handled);
// NULL static int (*subscribe_event)(struct v4l2_subdev *sd, struct v4l2_fh *fh, struct v4l2_event_subscription *sub);
// NULL static int (*unsubscribe_event)(struct v4l2_subdev *sd, struct v4l2_fh *fh, struct v4l2_event_subscription *sub);


static struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
    .g_chip_ident = ov5640_g_chip_ident,
    .log_status = NULL,
    .s_io_pin_config = NULL,
    .init = NULL,
    .load_fw = NULL,
    .reset = NULL,
    .s_gpio = NULL,
    .queryctrl = ov5640_queryctrl,
    .g_ctrl = NULL,
    .s_ctrl = NULL,
    .g_ext_ctrls = NULL,
    .s_ext_ctrls = NULL,
    .try_ext_ctrls = NULL,
    .querymenu = NULL,
    .g_std = NULL,
    .s_std = NULL,
    .ioctl = NULL,
#ifdef CONFIG_VIDEO_ADV_DEBUG
    .g_register = ov5640_g_register,
    .s_register = ov5640_s_register,
#endif
    .s_power        = ov5640_s_power,
    .interrupt_service_routine = NULL,
    .subscribe_event = NULL,
    .unsubscribe_event = NULL,
};


/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

// NULL static int ov5640_s_routing(struct v4l2_subdev *sd, u32 input, u32 output, u32 config);
// NULL static int ov5640_s_crystal_freq)(struct v4l2_subdev *sd, u32 freq, u32 flags);
// NULL static int ov5640_s_std_output)(struct v4l2_subdev *sd, v4l2_std_id std);
// NULL static int ov5640_g_std_output)(struct v4l2_subdev *sd, v4l2_std_id *std);
// NULL static int ov5640_querystd)(struct v4l2_subdev *sd, v4l2_std_id *std);
// NULL static int ov5640_g_tvnorms_output)(struct v4l2_subdev *sd, v4l2_std_id *std);
// NULL static int ov5640__input_status)(struct v4l2_subdev *sd, u32 *status);

static int ov5640_s_stream(struct v4l2_subdev * sd, int enable)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: start stream\n", __func__);
    return 0;
}

// NULL static int ov5640_cropcap)(struct v4l2_subdev *sd, struct v4l2_cropcap *cc);
// NULL static int ov5640_g_crop)(struct v4l2_subdev *sd, struct v4l2_crop *crop);
// NULL static int ov5640_s_crop)(struct v4l2_subdev *sd, struct v4l2_crop *crop);
// NULL static int ov5640_g_parm)(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
// NULL static int ov5640_s_parm)(struct v4l2_subdev *sd, struct v4l2_streamparm *param);
// NULL static int ov5640_g_frame_interval)(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval);
// NULL static int ov5640_s_frame_interval)(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *interval);
// NULL static int ov5640_enum_framesizes)(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize);
// NULL static int ov5640_enum_frameintervals)(struct v4l2_subdev *sd, struct v4l2_frmivalenum *fival);
// NULL static int ov5640_enum_dv_presets) (struct v4l2_subdev *sd, struct v4l2_dv_enum_preset *preset);
// NULL static int ov5640_s_dv_preset)(struct v4l2_subdev *sd, struct v4l2_dv_preset *preset);
// NULL static int ov5640_g_dv_preset)(struct v4l2_subdev *sd, struct v4l2_dv_preset *preset);
// NULL static int ov5640_query_dv_preset)(struct v4l2_subdev *sd, struct v4l2_dv_preset *preset);
// NULL static int ov5640_s_dv_timings)(struct v4l2_subdev *sd, struct v4l2_dv_timings *timings);
// NULL static int ov5640_g_dv_timings)(struct v4l2_subdev *sd, struct v4l2_dv_timings *timings);
// NULL static int ov5640_enum_dv_timings)(struct v4l2_subdev *sd, struct v4l2_enum_dv_timings *timings);
// NULL static int ov5640_query_dv_timings)(struct v4l2_subdev *sd, struct v4l2_dv_timings *timings);
// NULL static int ov5640_dv_timings_cap)(struct v4l2_subdev *sd, struct v4l2_dv_timings_cap *cap);

static int ov5640_enum_mbus_fmt(struct v4l2_subdev * sd, unsigned int index, enum v4l2_mbus_pixelcode *code)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: index %d\n", __func__, index);

    if (index >= ARRAY_SIZE(ov5640_color_formats))
        return -EINVAL;
    *code = ov5640_color_formats[index].code;

    return 0;
}

// NULL static int ov5640_enum_mbus_fsizes)(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize);

static int ov5640_g_mbus_fmt(struct v4l2_subdev * sd, struct v4l2_mbus_framefmt *fmt)
{
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: get mbus format\n", __func__);

    *fmt = info->format;

    return 0;
}

static int ov5640_try_mbus_fmt(struct v4l2_subdev * sd, struct v4l2_mbus_framefmt * fmt)
{
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);
    const ov5640_dataformat * try_fmt = ov5640_find_dataformat(fmt->code);

    dev_dbg(&i2c->dev, "[OV5640] %s: try mbus format\n", __func__);


    fmt->width = info->crop.width;
    fmt->height = info->crop.height;

    if (!try_fmt) {
        fmt->code    = try_fmt->code;
        fmt->colorspace  = try_fmt->colorspace;
    }

    fmt->field   = V4L2_FIELD_NONE;

    return 0;
}

static int ov5640_s_mbus_fmt(struct v4l2_subdev * sd, struct v4l2_mbus_framefmt * fmt)
{
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);
    const ov5640_dataformat * try_fmt = ov5640_find_dataformat(fmt->code);

    dev_dbg(&i2c->dev, "[OV5640] %s: set mbus format\n", __func__);


    if (ov5640_find_dataformat(fmt->code))
        return -EINVAL;

    ov5640_try_mbus_fmt(sd, fmt);

    if (try_fmt) {
        info->format.code = try_fmt->code;
        info->format.colorspace = try_fmt->colorspace;
    }
    else {
        return -EINVAL;
    }

    return 0;
}
// NULL static int ov5640_g_mbus_config)(struct v4l2_subdev *sd, struct v4l2_mbus_config *cfg);
// NULL static int ov5640_s_mbus_config)(struct v4l2_subdev *sd, const struct v4l2_mbus_config *cfg);

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
    .s_stream       = ov5640_s_stream,
    .enum_mbus_fmt = ov5640_enum_mbus_fmt,
    .g_mbus_fmt = ov5640_g_mbus_fmt,
    .try_mbus_fmt = ov5640_try_mbus_fmt,
    .s_mbus_fmt = ov5640_s_mbus_fmt,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev sensor operations
 */

struct v4l2_subdev_sensor_ops ov5640_subdev_sensor_ops = {
    .g_skip_top_lines = NULL,
    .g_skip_frames = NULL,
};


/* -----------------------------------------------------------------------------
 * V4L2 subdev pad operations
 */

static int ov5640_enum_mbus_code(struct v4l2_subdev * sd,
                  struct v4l2_subdev_fh * fh,
                  struct v4l2_subdev_mbus_code_enum * code)
{
    //struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: enum mbus code\n", __func__);

    if(code->index >= ARRAY_SIZE(ov5640_color_formats)) {
        return -EINVAL;
    }

    code->code = ov5640_color_formats[code->index].code;

    return 0;
}

static int ov5640_enum_frame_size(struct v4l2_subdev * sd,
                   struct v4l2_subdev_fh * fh,
                   struct v4l2_subdev_frame_size_enum * fse)
{
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: enum frame size\n", __func__);

    if (fse->index >= OV5640_SIZE_LAST || fse->code != info->format.code)
        return -EINVAL;

    fse->min_width = ov5640_frame_sizes[fse->index].width;
    fse->max_width = fse->min_width;
    fse->min_height = ov5640_frame_sizes[fse->index].height;
    fse->max_height = fse->min_height;

    return 0;
}

static int ov5640_enum_frame_interval(struct v4l2_subdev * sd,
               struct v4l2_subdev_fh * fh,
               struct v4l2_subdev_frame_interval_enum * fie)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: enum frame interval\n", __func__);
    return 0;
}

static struct v4l2_mbus_framefmt *
__ov5640_get_pad_format(struct ov5640 * info, struct v4l2_subdev_fh * fh,
            unsigned int pad, u32 which)
{
    dev_dbg(NULL, "[OV5640] %s: get pad format\n", __func__);

    switch (which) {
        case V4L2_SUBDEV_FORMAT_TRY:
            return v4l2_subdev_get_try_format(fh, pad);
        case V4L2_SUBDEV_FORMAT_ACTIVE:
            return &info->format;
        default:
            return NULL;
    }
}

static int ov5640_get_fmt(struct v4l2_subdev * sd,
                  struct v4l2_subdev_fh * fh,
                  struct v4l2_subdev_format * fmt)
{
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: get format\n", __func__);

    fmt->format = *(__ov5640_get_pad_format(info, fh, fmt->pad, fmt->which));

    return 0;
}

static int ov5640_set_fmt(struct v4l2_subdev * sd,
                  struct v4l2_subdev_fh * fh,
                  struct v4l2_subdev_format * format)
{
    struct v4l2_frmsize_discrete size;
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: set format\n", __func__);

    size.height     = format->format.height;
    size.width  = format->format.width; 
    info->res_index       = ov5640_v4l2_try_fmt_cap(&size);
    info->crop.width      = size.width;
    info->crop.height     = size.height;
    info->curr_crop.width     = size.width;
    info->curr_crop.height    = size.height;
    info->format.width        = size.width;
    info->format.height       = size.height;
    info->format.code         = V4L2_MBUS_FMT_SBGGR10_1X10;

    format->format.width        = size.width;
    format->format.height       = size.height;

    return 0;
}

static int ov5640_get_crop(struct v4l2_subdev * sd,
                struct v4l2_subdev_fh *fh,
                struct v4l2_subdev_crop *crop)
{
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: get crop\n", __func__);

    crop->rect.left = info->crop.left;
    crop->rect.top = info->crop.top;
    crop->rect.width      = info->crop.width;
    crop->rect.height     = info->crop.height;
    crop->which         = 0;

    return 0;
}

static int ov5640_set_crop(struct v4l2_subdev * sd,
                struct v4l2_subdev_fh * fh,
                struct v4l2_subdev_crop * crop)
{
    struct v4l2_frmsize_discrete size;
    struct ov5640 * info = to_ov5640(sd);
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: set crop\n", __func__);

    size.height     = crop->rect.height;
    size.width  = crop->rect.width; 
    info->res_index       = ov5640_v4l2_try_fmt_cap(&size);
    info->crop.width      = size.width;
    info->crop.height     = size.height;
    info->curr_crop.width     = size.width;
    info->curr_crop.height    = size.height;
    info->format.width        = size.width;
    info->format.height       = size.height;
    info->format.code         = V4L2_MBUS_FMT_SBGGR10_1X10;   

    crop->rect.width        = size.width;
    crop->rect.height       = size.height;

    return 0;
}

// NULL static int ov5640_get_selection)(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_selection *sel);
// NULL static int ov5640_set_selection)(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_selection *sel);

#ifdef CONFIG_MEDIA_CONTROLLER
// NULL static int ov5640_link_validate)(struct v4l2_subdev *sd, struct media_link *link, struct v4l2_subdev_format *source_fmt,s struct v4l2_subdev_format *sink_fmt);
#endif /* CONFIG_MEDIA_CONTROLLER */


static struct v4l2_subdev_pad_ops ov5640_subdev_pad_ops = {
    .enum_mbus_code = ov5640_enum_mbus_code,
    .enum_frame_size = ov5640_enum_frame_size,
    .enum_frame_interval = ov5640_enum_frame_interval,
    .get_fmt = ov5640_get_fmt,
    .set_fmt = ov5640_set_fmt,
    .set_crop = ov5640_set_crop,
    .get_crop = ov5640_get_crop,
    .get_selection = NULL,
    .set_selection = NULL,
    .link_validate = NULL,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static int ov5640_registered(struct v4l2_subdev * sd)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);
    struct ov5640 * info = to_ov5640(sd);
    int ret;

    dev_dbg(&i2c->dev, "[OV5640] %s: registered\n", __func__);

    ret = ov5640_power_on(info);
    if (ret < 0) {
        dev_err(&i2c->dev, "OV5640 power up failed\n");
        return ret;
    }

    ret = ov5640_power_off(info);
    return 0;
}

static int ov5640_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);
    struct ov5640 * info = to_ov5640(sd);
    int ret = 0;

    dev_dbg(&i2c->dev, "[OV5640] %s: open\n", __func__);

    info->crop.width = ov5640_frame_sizes[OV5640_SIZE_FULL].width;
    info->crop.height = ov5640_frame_sizes[OV5640_SIZE_FULL].height;
    info->crop.left = 0;
    info->crop.top = 0;

    info->format.code = V4L2_MBUS_FMT_SGRBG12_1X12;
    info->format.width = info->crop.width;
    info->format.height = info->crop.height;
    info->format.field = V4L2_FIELD_NONE;
    info->format.colorspace = V4L2_COLORSPACE_SRGB;

    ret = ov5640_s_power(sd, 1);

    ov5640_write_array(i2c, init_table);
    return 0;
}

static int ov5640_close(struct v4l2_subdev * sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client * i2c = v4l2_get_subdevdata(sd);

    dev_dbg(&i2c->dev, "[OV5640] %s: close\n", __func__);

    return ov5640_s_power(sd, 0);
}

static const struct v4l2_subdev_internal_ops ov5640_subdev_internal_ops = {
    .registered = ov5640_registered,
    .unregistered = NULL,
    .open = ov5640_open,
    .close = ov5640_close,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
    .core   = &ov5640_subdev_core_ops,
    .video  = &ov5640_subdev_video_ops,
    .sensor = &ov5640_subdev_sensor_ops,
    .pad    = &ov5640_subdev_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */

static int ov5640_probe(struct i2c_client * i2c, const struct i2c_device_id * did)
{
    struct ov5640_platform_data * pdata = i2c->dev.platform_data;
    struct i2c_adapter *adapter = to_i2c_adapter(i2c->dev.parent);
    struct ov5640 * ov5640;
    int i;
    int ret = 0;

    dev_dbg(&i2c->dev, "[OV5640] %s: probe\n", __func__);

    if (pdata == NULL) {
        dev_err(&i2c->dev, "No platform data\n");
        return -EINVAL;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
        dev_warn(&i2c->dev,
            "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
        return -EIO;
    }

    ov5640 = kzalloc(sizeof(*ov5640), GFP_KERNEL);
    if (ov5640 == NULL)
        return -ENOMEM;

    ov5640->pdata = pdata;
    ov5640->model = did->driver_data;
    ov5640->reset = -1;

    v4l2_ctrl_handler_init(&ov5640->ctrls, ARRAY_SIZE(ov5640_ctrls) + 4);

    for (i = 0; i < ARRAY_SIZE(ov5640_ctrls); ++i) {
        if (ov5640_ctrls[i].name)
            v4l2_ctrl_new_custom(&ov5640->ctrls, &ov5640_ctrls[i], NULL);

        if (ov5640->ctrls.error) {
            dev_warn(&i2c->dev, "%s: control initializ %s error %d\n",
                   __func__, ov5640_ctrls[i].name, ov5640->ctrls.error);
            ret = ov5640->ctrls.error;
            goto done;
        }
    }

    ov5640->subdev.ctrl_handler = &ov5640->ctrls;

    if (ov5640->ctrls.error) {
        printk(KERN_INFO "%s: control initialization error %d\n",
               __func__, ov5640->ctrls.error);
        ret = ov5640->ctrls.error;
        goto done;
    }

    //ov5640->blc_auto = v4l2_ctrl_find(&ov5640->ctrls, V4L2_CID_BLC_AUTO);
    //ov5640->blc_offset = v4l2_ctrl_find(&ov5640->ctrls,
    //                     V4L2_CID_BLC_DIGITAL_OFFSET);

    mutex_init(&ov5640->power_lock);
    v4l2_i2c_subdev_init(&ov5640->subdev, i2c, &ov5640_subdev_ops);
    ov5640->subdev.internal_ops = &ov5640_subdev_internal_ops;

    ov5640->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_init(&ov5640->subdev.entity, 1, &ov5640->pad, 0);
    if (ret < 0)
        goto done;

    ov5640->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    ov5640->crop.width = ov5640_frame_sizes[OV5640_SIZE_FULL].width;
    ov5640->crop.height = ov5640_frame_sizes[OV5640_SIZE_FULL].height;
    ov5640->crop.left = 0;
    ov5640->crop.top = 0;

    ov5640->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;

    ov5640->format.width = ov5640->crop.width;
    ov5640->format.height = ov5640->crop.height;
    ov5640->format.field = V4L2_FIELD_NONE;
    ov5640->format.colorspace = V4L2_COLORSPACE_SRGB;

#if 0
    if (pdata->reset != -1) {
        ret = gpio_request_one(pdata->reset, GPIOF_OUT_INIT_LOW,
                       "ov5640_rst");
        if (ret < 0)
            goto done;

        ov5640->reset = pdata->reset;
    }
#endif

    //ret = ov5640_pll_setup(ov5640);

done:
    if (ret < 0) {
#if 0
        if (ov5640->reset != -1)
            gpio_free(ov5640->reset);
#endif

        v4l2_ctrl_handler_free(&ov5640->ctrls);
        media_entity_cleanup(&ov5640->subdev.entity);
        kfree(ov5640);
    }

    return ret;
}

static int ov5640_remove(struct i2c_client * i2c)
{
    struct v4l2_subdev * subdev = i2c_get_clientdata(i2c);
    struct ov5640 * ov5640 = to_ov5640(subdev);

    dev_dbg(&i2c->dev, "[OV5640] %s: remove driver\n", __func__);

    v4l2_ctrl_handler_free(&ov5640->ctrls);
    v4l2_device_unregister_subdev(subdev);
    media_entity_cleanup(&subdev->entity);
    if (ov5640->reset != -1)
        gpio_free(ov5640->reset);
    kfree(ov5640);

    return 0;
}

static const struct i2c_device_id ov5640_id[] = {
    { "ov5640", OV5640_MODEL_COLOR },
    { "ov5640m", OV5640_MODEL_MONOCHROME },
    { }
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
    .driver = {
        .name = "ov5640",
    },
    .probe          = ov5640_probe,
    .remove         = ov5640_remove,
    .id_table       = ov5640_id,
};

module_i2c_driver(ov5640_i2c_driver);

MODULE_DESCRIPTION("OmniVision OV5640 Camera driver");
MODULE_AUTHOR("Alex Lei <alexvonduar@gmail.com>");
MODULE_LICENSE("GPL v2");
