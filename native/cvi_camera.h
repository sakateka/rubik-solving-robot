#ifndef RUBIK_CVI_CAMERA_H
#define RUBIK_CVI_CAMERA_H

#include <stdint.h>

typedef struct RubikCamera RubikCamera;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint32_t stride[3];
    uint32_t length[3];
} RubikCameraFrameInfo;

// Opens the sensor through CVI's VI/ISP sample helpers. `sensor_config` is a
// path to sensor_cfg.ini. Returns NULL on error and writes a short diagnostic.
RubikCamera *rubik_camera_open(const char *sensor_config, char *error, uint32_t error_len);

// Acquires and immediately releases one VI frame. It deliberately does not
// expose the frame memory yet: this probe verifies camera initialisation before
// introducing VPSS crop/resize and TPU tensor ownership.
int rubik_camera_probe_frame(RubikCamera *camera, RubikCameraFrameInfo *info,
                             char *error, uint32_t error_len);

// Acquires one frame after VPSS crops the fixed cube ROI and resizes it to
// 320x320 RGB planar. The caller receives metadata only at this stage.
int rubik_camera_probe_vpss_frame(RubikCamera *camera, RubikCameraFrameInfo *info,
                                  char *error, uint32_t error_len);

// Discards initial VPSS frames while sensor streaming and 3A converge.
int rubik_camera_warmup_vpss(RubikCamera *camera, uint32_t frame_count,
                              char *error, uint32_t error_len);

// Captures the VPSS RGB-planar frame into `output` as tightly packed CHW u8.
// This exists for validation and for the later f32 normalisation step.
int rubik_camera_copy_vpss_rgb(RubikCamera *camera, uint8_t *output,
                                uint32_t output_len, RubikCameraFrameInfo *info,
                                char *error, uint32_t error_len);

void rubik_camera_close(RubikCamera *camera);

#endif
