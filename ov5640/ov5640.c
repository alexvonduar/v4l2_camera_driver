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

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ov5640.h"

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

struct ov5640 {
    struct v4l2_subdev subdev;
    struct media_pad pad;
    struct v4l2_rect crop;  /* Sensor window */
    struct v4l2_mbus_framefmt format;
    struct ov5640_platform_data *pdata;
    struct mutex power_lock; /* lock to protect power_count */
    int power_count;

    enum ov5640_model model;
    //struct aptina_pll pll;
    int reset;
    int pwdn;

    struct v4l2_ctrl_handler ctrls;
    struct v4l2_ctrl *blc_auto;
    struct v4l2_ctrl *blc_offset;
};

static struct ov5640 *to_ov5640(struct v4l2_subdev *sd)
{
    return container_of(sd, struct ov5640, subdev);
}

static int ov5640_read(struct i2c_client *client, u8 reg)
{
    return i2c_smbus_read_word_swapped(client, reg);
}

static int ov5640_write(struct i2c_client *client, u8 reg, u16 data)
{
    return i2c_smbus_write_word_swapped(client, reg, data);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

static int ov5640_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *chip)
{
    return 0;
}

// NULL static int ov5640_log_status(struct v4l2_subdev *sd);
// NULL static int ov5640_s_io_pin_config(struct v4l2_subdev *sd, size_t n, struct v4l2_subdev_io_pin_config *pincfg);
// NULL static int ov5640_init(struct v4l2_subdev *sd, u32 val);
// NULL static int ov5640_load_fw(struct v4l2_subdev *sd);
// NULL static int ov5640_reset(struct v4l2_subdev *sd, u32 val);
// NULL static int ov5640_s_gpio(struct v4l2_subdev *sd, u32 val);

static int ov5640_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
    return 0;
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
    return 0;
}

static int ov5640_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
    return 0;
}
#endif
static int ov5640_s_power(struct v4l2_subdev *sd, int on)
{
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
static int ov5640_s_stream(struct v4l2_subdev *subdev, int enable)
{
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
// NULL static int ov5640_enum_mbus_fmt)(struct v4l2_subdev *sd, unsigned int index, enum v4l2_mbus_pixelcode *code);
// NULL static int ov5640_enum_mbus_fsizes)(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize);
// NULL static int ov5640_g_mbus_fmt)(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt);
// NULL static int ov5640_try_mbus_fmt)(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt);
// NULL static int ov5640_s_mbus_fmt)(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt);
// NULL static int ov5640_g_mbus_config)(struct v4l2_subdev *sd, struct v4l2_mbus_config *cfg);
// NULL static int ov5640_s_mbus_config)(struct v4l2_subdev *sd, const struct v4l2_mbus_config *cfg);

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
    .s_stream       = ov5640_s_stream,
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

static int ov5640_enum_mbus_code(struct v4l2_subdev *subdev,
                  struct v4l2_subdev_fh *fh,
                  struct v4l2_subdev_mbus_code_enum *code)
{
    return 0;
}

static int ov5640_enum_frame_size(struct v4l2_subdev *subdev,
                   struct v4l2_subdev_fh *fh,
                   struct v4l2_subdev_frame_size_enum *fse)
{
    return 0;
}

static int ov5640_enum_frame_interval(struct v4l2_subdev *sd,
               struct v4l2_subdev_fh *fh,
               struct v4l2_subdev_frame_interval_enum *fie)
{
    return 0;
}

static int ov5640_get_fmt(struct v4l2_subdev *subdev,
                  struct v4l2_subdev_fh *fh,
                  struct v4l2_subdev_format *fmt)
{
    return 0;
}

static int ov5640_set_fmt(struct v4l2_subdev *subdev,
                  struct v4l2_subdev_fh *fh,
                  struct v4l2_subdev_format *format)
{
    return 0;
}

static int ov5640_get_crop(struct v4l2_subdev *subdev,
                struct v4l2_subdev_fh *fh,
                struct v4l2_subdev_crop *crop)
{
    return 0;
}

static int ov5640_set_crop(struct v4l2_subdev *subdev,
                struct v4l2_subdev_fh *fh,
                struct v4l2_subdev_crop *crop)
{
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

static int ov5640_registered(struct v4l2_subdev *subdev)
{
    return 0;
}

static int ov5640_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
    return 0;
}

static int ov5640_close(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
    return 0;
}

static const struct v4l2_subdev_internal_ops ov5640_subdev_internal_ops = {
    .registered = ov5640_registered,
    .unregistered = NULL,
    .open = ov5640_open,
    .close = ov5640_close,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev control operations
 */
 
static int ov5640_s_ctrl(struct v4l2_ctrl *ctrl)
{
    return 0;
}

static struct v4l2_ctrl_ops ov5640_ctrl_ops = {
    .s_ctrl = ov5640_s_ctrl,
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

static int ov5640_probe(struct i2c_client *client,
             const struct i2c_device_id *did)
{
    struct ov5640_platform_data *pdata = client->dev.platform_data;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ov5640 *ov5640;
    //unsigned int i;
    int ret = 0;

    if (pdata == NULL) {
        dev_err(&client->dev, "No platform data\n");
        return -EINVAL;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
        dev_warn(&client->dev,
            "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
        return -EIO;
    }

    ov5640 = kzalloc(sizeof(*ov5640), GFP_KERNEL);
    if (ov5640 == NULL)
        return -ENOMEM;

    ov5640->pdata = pdata;
    ov5640->model = did->driver_data;
    ov5640->reset = -1;

    //v4l2_ctrl_handler_init(&ov5640->ctrls, ARRAY_SIZE(ov5640_ctrls) + 4);

    v4l2_ctrl_new_std(&ov5640->ctrls, &ov5640_ctrl_ops,
              V4L2_CID_HFLIP, 0, 1, 1, 0);
    v4l2_ctrl_new_std(&ov5640->ctrls, &ov5640_ctrl_ops,
              V4L2_CID_VFLIP, 0, 1, 1, 0);

    //for (i = 0; i < ARRAY_SIZE(ov5640_ctrls); ++i)
    //    v4l2_ctrl_new_custom(&ov5640->ctrls, &ov5640_ctrls[i], NULL);

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
    v4l2_i2c_subdev_init(&ov5640->subdev, client, &ov5640_subdev_ops);
    ov5640->subdev.internal_ops = &ov5640_subdev_internal_ops;

    ov5640->pad.flags = MEDIA_PAD_FL_SOURCE;
    //ret = media_entity_init(&ov5640->subdev.entity, 1, &ov5640->pad, 0);
    //if (ret < 0)
    //    goto done;

    ov5640->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    //ov5640->crop.width = OV5640_WINDOW_WIDTH_DEF;
    //ov5640->crop.height = OV5640_WINDOW_HEIGHT_DEF;
    //ov5640->crop.left = OV5640_COLUMN_START_DEF;
    //ov5640->crop.top = OV5640_ROW_START_DEF;

    if (ov5640->model == OV5640_MODEL_MONOCHROME)
        ov5640->format.code = V4L2_MBUS_FMT_Y12_1X12;
    else
        ov5640->format.code = V4L2_MBUS_FMT_SGRBG12_1X12;

    //ov5640->format.width = OV5640_WINDOW_WIDTH_DEF;
    //ov5640->format.height = OV5640_WINDOW_HEIGHT_DEF;
    ov5640->format.field = V4L2_FIELD_NONE;
    ov5640->format.colorspace = V4L2_COLORSPACE_SRGB;

    //if (pdata->reset != -1) {
    //    ret = gpio_request_one(pdata->reset, GPIOF_OUT_INIT_LOW,
    //                   "ov5640_rst");
    //    if (ret < 0)
    //        goto done;

    //    ov5640->reset = pdata->reset;
    //}

    //ret = ov5640_pll_setup(ov5640);

done:
    if (ret < 0) {
        if (ov5640->reset != -1)
            gpio_free(ov5640->reset);

        v4l2_ctrl_handler_free(&ov5640->ctrls);
        media_entity_cleanup(&ov5640->subdev.entity);
        kfree(ov5640);
    }

    return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
    struct v4l2_subdev *subdev = i2c_get_clientdata(client);
    struct ov5640 *ov5640 = to_ov5640(subdev);

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
