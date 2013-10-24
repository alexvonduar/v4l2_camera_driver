#ifndef _MEDIA_OV5640_H
#define _MEDIA_OV5640_H

struct v4l2_subdev;

struct ov5640_platform_data {
    unsigned int clk_pol:1;

    void (*set_clock)(struct v4l2_subdev *subdev, unsigned int rate);

    const s64 *link_freqs;
    s64 link_def_freq;
};

#endif
