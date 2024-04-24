// SPDX-License-Identifier: GPL-2.0
/*
 * DRM driver for Ilitek ILI9488 panels
 *
 * Copyright 2020 Kamlesh Gurudasani <kamlesh.gurudasani@gmail.com>
 */

#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

#define ILI9488_VCOM_CONTROL_1			0xC5
#define ILI9488_COLUMN_ADDRESS_SET		0x2A
#define ILI9488_PAGE_ADDRESS_SET		0x2B
#define ILI9488_MEMORY_WRITE			0x2C
#define ILI9488_POSITIVE_GAMMA_CORRECTION	0xE0
#define ILI9488_NEGATIVE_GAMMA_CORRECTION	0xE1
#define ILI9488_POWER_CONTROL_1			0xC0
#define ILI9488_POWER_CONTROL_2			0xC1

#define ILI9488_MEMORY_ACCESS_CONTROL		0x36
#define ILI9488_COLMOD_PIXEL_FORMAT_SET		0x3A
#define ILI9488_INTERFACE_MODE_CONTROL		0xB0
#define ILI9488_FRAME_RATE_CONTROL_PARTIAL	0xB3
#define ILI9488_DISPLAY_INVERSION_CONTROL	0xB4
#define ILI9488_SET_IMAGE_FUNCTION		0xE9
#define ILI9488_ADJUST_CONTROL_3		0xF7
#define ILI9488_ADJUST_CONTROL_3		0xF7
#define ILI9488_DISPLAY_ON			0x29
#define ILI9488_DISPLAY_OFF			0x28
#define ILI9488_ENTER_SLEEP_MODE		0x10
#define ILI9488_DBI_BPP18			0x06
#define ILI9488_DPI_BPP18			0x60
#define ILI9488_FRAME_RATE_CONTROL_NORMAL	0xB1
#define ILI9488_SLEEP_OUT			0x11

#define ILI9488_MADCTL_BGR	BIT(3)
#define ILI9488_MADCTL_MV	BIT(5)
#define ILI9488_MADCTL_MX	BIT(6)
#define ILI9488_MADCTL_MY	BIT(7)

static void ili9488_rgb565_to_rgb666_line(u8 *dst, u16 *sbuf,
					  unsigned int pixels)
{
	unsigned int x;
for (x = 0; x < pixels; x+=2) {

    u16 w0 = (*sbuf);
    sbuf++;
   //   u16 w1 = (*(sbuf+1)); // error
    u16 w1 = (*sbuf); // corrected by firstman
    sbuf++;

    u8 r0 = ((w0 & 0xF800) >> 8);
    u8 g0 = ((w0 & 0x07E0) >> 3);
    u8 b0 = ((w0 & 0x001F) << 3);

    u8 r1 = ((w1 & 0xF800) >> 8);
    u8 g1 = ((w1 & 0x07E0) >> 3);
    u8 b1 = ((w1 & 0x001F) << 3);

    *dst++ = g0; // G0
    *dst++ = r0; // R0

    *dst++ = r1; // R1
    *dst++ = b0; // B0

    *dst++ = b1; // B1
    *dst++ = g1; // G1.
}
}

static void ili9488_rgb565_to_rgb666(u8 *dst, void *vaddr,
				     struct drm_framebuffer *fb,
				     struct drm_rect *rect)
{
	size_t linepixels = rect->x2 - rect->x1;
	size_t src_len = linepixels * sizeof(u16);
	size_t dst_len = linepixels * 3;
	unsigned int y, lines = rect->y2 - rect->y1;
	u16 *sbuf;

	/*
	 * The cma memory is write-combined so reads are uncached.
	 * Speed up by fetching one line at a time.
	 */
	sbuf = kmalloc(src_len, GFP_KERNEL);
	if (!sbuf)
		return;

	vaddr += rect->y1 * fb->pitches[0] + rect->x1 * sizeof(u16);
	for (y = 0; y < lines; y++) {
		memcpy(sbuf, vaddr, src_len);
		ili9488_rgb565_to_rgb666_line(dst, sbuf, linepixels);
		vaddr += fb->pitches[0];
		dst += dst_len;
	}
	kfree(sbuf);
}

static int ili9488_buf_copy(void *dst, struct drm_framebuffer *fb,
			    struct drm_rect *rect)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct dma_buf_attachment *import_attach = cma_obj->base.import_attach;
	struct drm_format_name_buf format_name;
	void *src = cma_obj->vaddr;
	int ret = 0;

	if (import_attach) {
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
					       DMA_FROM_DEVICE);
		if (ret)
			return ret;
	}

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		ili9488_rgb565_to_rgb666(dst, src, fb, rect);
		break;
	default:
		dev_err_once(fb->dev->dev, "Format is not supported: %s\n",
			     drm_get_format_name(fb->format->format,
						 &format_name));
		return -EINVAL;
	}

	if (import_attach)
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
					     DMA_FROM_DEVICE);
	return ret;
}

static void ili9488_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int idx, ret = 0;
	void *tr;
	bool full;
	unsigned int height = rect->y2 - rect->y1;
	unsigned int width = rect->x2 - rect->x1;

	// if (!dbidev->enabled)
	// 	return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	full = width == fb->width && height == fb->height;

	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id,
		      DRM_RECT_ARG(rect));

	/* Always invoke copy buffer routine as the display supports
	 * only RGB666 format which is not implemented in DRM
	 */
	if (!dbi->dc || !full ||
	    fb->format->format == DRM_FORMAT_RGB565) {
		tr = dbidev->tx_buf;
		ret = ili9488_buf_copy(dbidev->tx_buf, fb, rect);
		if (ret)
			goto err_msg;
	} else {
		tr = cma_obj->vaddr;
	}

	mipi_dbi_command(dbi, ILI9488_COLUMN_ADDRESS_SET,
			 (rect->x1 >> 8) & 0xFF, rect->x1 & 0xFF,
			 (rect->x2 >> 8) & 0xFF, (rect->x2 - 1) & 0xFF);

	mipi_dbi_command(dbi, ILI9488_PAGE_ADDRESS_SET,
			 (rect->y1 >> 8) & 0xFF, rect->y1 & 0xFF,
			 (rect->y2 >> 8) & 0xFF, (rect->y2 - 1) & 0xFF);

	ret = mipi_dbi_command_buf(dbi, ILI9488_MEMORY_WRITE, tr,
				   width * height * 3);

 err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);

	drm_dev_exit(idx);
}

static void ili9488_pipe_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect rect;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		ili9488_fb_dirty(state->fb, &rect);
}

static void ili9488_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct drm_framebuffer *fb = plane_state->fb;
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;
	struct drm_rect rect = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	mipi_dbi_command(dbi, ILI9488_POSITIVE_GAMMA_CORRECTION,
			 0x00, 0x03, 0x09, 0x08, 0x16,
			 0x0a, 0x3f, 0x78, 0x4c, 0x09,
			 0x0a, 0x08, 0x16, 0x1a, 0x0f);

	mipi_dbi_command(dbi, ILI9488_NEGATIVE_GAMMA_CORRECTION,
			 0x00, 0x16, 0x19, 0x03, 0x0f,
			 0x05, 0x32, 0x45, 0x46, 0x04,
			 0x0e, 0x0d, 0x35, 0x37, 0x0f);

	mipi_dbi_command(dbi, ILI9488_POWER_CONTROL_1, 0x17, 0x15);

	mipi_dbi_command(dbi, ILI9488_POWER_CONTROL_2, 0x41);

	mipi_dbi_command(dbi, ILI9488_VCOM_CONTROL_1, 0x00, 0x12, 0x80);

	mipi_dbi_command(dbi, ILI9488_COLMOD_PIXEL_FORMAT_SET,
			 ILI9488_DBI_BPP18 | ILI9488_DPI_BPP18);

	mipi_dbi_command(dbi, ILI9488_INTERFACE_MODE_CONTROL, 0x80);

	mipi_dbi_command(dbi, ILI9488_FRAME_RATE_CONTROL_NORMAL, 0xa0);

	mipi_dbi_command(dbi, ILI9488_DISPLAY_INVERSION_CONTROL, 0x02);

	mipi_dbi_command(dbi, ILI9488_SET_IMAGE_FUNCTION, 0x00);

	mipi_dbi_command(dbi, ILI9488_ADJUST_CONTROL_3,
			 0xa9, 0x51, 0x2c, 0x82);

	mipi_dbi_command(dbi, ILI9488_SLEEP_OUT);

	msleep(120);

	mipi_dbi_command(dbi, ILI9488_DISPLAY_ON);

	// dbidev->enabled = true;
	ili9488_fb_dirty(fb, &rect);

 out_enable:
	switch (dbidev->rotation) {
	default:
		addr_mode = ILI9488_MADCTL_MX;
		break;
	case 90:
		addr_mode = ILI9488_MADCTL_MV;
		break;
	case 180:
		addr_mode = ILI9488_MADCTL_MY;
		break;
	case 270:
		addr_mode = ILI9488_MADCTL_MV | ILI9488_MADCTL_MY |
			ILI9488_MADCTL_MX;
		break;
	}
	addr_mode |= ILI9488_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
 out_exit:
	drm_dev_exit(idx);
}

static void ili9488_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);

	/*
	 * This callback is not protected by drm_dev_enter/exit since we want to
	 * turn off the display on regular driver unload. It's highly unlikely
	 * that the underlying SPI controller is gone should this be called
	 * after unplug.
	 */

	DRM_DEBUG_KMS("\n");

	// if (!dbidev->enabled)
	// 	return;

	mipi_dbi_command(&dbidev->dbi, MIPI_DCS_SET_DISPLAY_OFF);
	// dbidev->enabled = false;
}

static const u32 ili9488_formats[] = {
	DRM_FORMAT_RGB565,
};

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
	.enable = ili9488_pipe_enable,
	.disable = ili9488_pipe_disable,
	.update = ili9488_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode ili9488_mode = {
	DRM_SIMPLE_MODE(320, 480, 49, 73),
};

DEFINE_DRM_GEM_CMA_FOPS(ili9488_fops);

static struct drm_driver ili9488_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ili9488_fops,
	DRM_GEM_CMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "ili9488",
	.desc			= "Ilitek ILI9488",
	.date			= "20200607",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id ili9488_of_match[] = {
	{ .compatible = "eastrising,er-tft035-6" },
	{ }
};
MODULE_DEVICE_TABLE(of, ili9488_of_match);

static const struct spi_device_id ili9488_id[] = {
	{ "er-tft035-6", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ili9488_id);

static int ili9488_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	size_t bufsize;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &ili9488_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	// ret = devm_drm_dev_init(dev, drm, &ili9488_driver);
	// if (ret) {
	// 	kfree(dbidev);
	// 	return ret;
	// }

	drm_mode_config_init(drm);

	bufsize = ili9488_mode.vdisplay * ili9488_mode.hdisplay * 3;

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	dbidev->drm.mode_config.preferred_depth = 16;

	ret = mipi_dbi_dev_init_with_formats(dbidev, &ili9488_pipe_funcs,
					     ili9488_formats,
					     ARRAY_SIZE(ili9488_formats),
					     &ili9488_mode, rotation, bufsize);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int ili9488_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void ili9488_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver ili9488_spi_driver = {
	.driver = {
		.name = "ili9488",
		.owner = THIS_MODULE,
		.of_match_table = ili9488_of_match,
	},
	.id_table = ili9488_id,
	.probe = ili9488_probe,
	.remove = ili9488_remove,
	.shutdown = ili9488_shutdown,
};
module_spi_driver(ili9488_spi_driver);

MODULE_DESCRIPTION("Ilitek ILI9488 DRM driver");
MODULE_AUTHOR("Kamlesh Gurudasani <kamlesh.gurudasani@gmail.com>");
MODULE_LICENSE("GPL");
