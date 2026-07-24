#include "cvi_camera.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sample_comm.h"

struct RubikCamera {
    SAMPLE_VI_CONFIG_S vi_config;
    CVI_BOOL system_ready;
    CVI_BOOL vi_ready;
};

static void set_error(char *error, uint32_t error_len, const char *format, ...) {
    if (error == NULL || error_len == 0) {
        return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(error, error_len, format, args);
    va_end(args);
}

RubikCamera *rubik_camera_open(const char *sensor_config, char *error, uint32_t error_len) {
    RubikCamera *camera = calloc(1, sizeof(*camera));
    if (camera == NULL) {
        set_error(error, error_len, "camera allocation failed");
        return NULL;
    }

    CVI_S32 ret = SAMPLE_COMM_VI_SetIniPath(sensor_config);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "SAMPLE_COMM_VI_SetIniPath failed: %#x", ret);
        goto fail;
    }

    SAMPLE_INI_CFG_S ini_config;
    memset(&ini_config, 0, sizeof(ini_config));
    ret = SAMPLE_COMM_VI_ParseIni(&ini_config);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "SAMPLE_COMM_VI_ParseIni failed: %#x", ret);
        goto fail;
    }

    ret = CVI_VI_SetDevNum(ini_config.devNum);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "CVI_VI_SetDevNum failed: %#x", ret);
        goto fail;
    }

    ret = SAMPLE_COMM_VI_IniToViCfg(&ini_config, &camera->vi_config);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "SAMPLE_COMM_VI_IniToViCfg failed: %#x", ret);
        goto fail;
    }
    if (camera->vi_config.s32WorkingViNum != 1) {
        set_error(error, error_len, "expected one configured VI device, got %d",
                  camera->vi_config.s32WorkingViNum);
        goto fail;
    }

    PIC_SIZE_E picture_size;
    SIZE_S size;
    ret = SAMPLE_COMM_VI_GetSizeBySensor(
        ini_config.enSnsType[0], &picture_size);
    if (ret == CVI_SUCCESS) {
        ret = SAMPLE_COMM_SYS_GetPicSize(picture_size, &size);
    }
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "could not derive sensor frame size: %#x", ret);
        goto fail;
    }

    ret = SAMPLE_PLAT_SYS_INIT(size);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "SAMPLE_PLAT_SYS_INIT failed: %#x", ret);
        goto fail;
    }
    camera->system_ready = CVI_TRUE;

    ret = SAMPLE_PLAT_VI_INIT(&camera->vi_config);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "SAMPLE_PLAT_VI_INIT failed: %#x", ret);
        goto fail;
    }
    camera->vi_ready = CVI_TRUE;
    return camera;

fail:
    rubik_camera_close(camera);
    return NULL;
}

int rubik_camera_probe_frame(RubikCamera *camera, RubikCameraFrameInfo *info,
                             char *error, uint32_t error_len) {
    if (camera == NULL || !camera->vi_ready || info == NULL) {
        set_error(error, error_len, "camera is not initialized");
        return -1;
    }

    VIDEO_FRAME_INFO_S frame;
    memset(&frame, 0, sizeof(frame));
    CVI_S32 ret = CVI_VI_GetChnFrame(0, 0, &frame, 3000);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "CVI_VI_GetChnFrame failed: %#x", ret);
        return -1;
    }

    info->width = frame.stVFrame.u32Width;
    info->height = frame.stVFrame.u32Height;
    info->pixel_format = (uint32_t)frame.stVFrame.enPixelFormat;
    for (int plane = 0; plane < 3; ++plane) {
        info->stride[plane] = frame.stVFrame.u32Stride[plane];
        info->length[plane] = frame.stVFrame.u32Length[plane];
    }

    ret = CVI_VI_ReleaseChnFrame(0, 0, &frame);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "CVI_VI_ReleaseChnFrame failed: %#x", ret);
        return -1;
    }
    return 0;
}

void rubik_camera_close(RubikCamera *camera) {
    if (camera == NULL) {
        return;
    }
    if (camera->vi_ready) {
        SAMPLE_COMM_VI_DestroyIsp(&camera->vi_config);
        SAMPLE_COMM_VI_DestroyVi(&camera->vi_config);
    }
    if (camera->system_ready) {
        SAMPLE_COMM_SYS_Exit();
    }
    free(camera);
}
