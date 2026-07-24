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
    CVI_BOOL vpss_ready;
    CVI_BOOL vpss_bound;
};

enum {
    RUBIK_VPSS_GRP = 0,
    RUBIK_VPSS_CHN = 0,
    RUBIK_ROI_X = 464,
    RUBIK_ROI_Y = 32,
    RUBIK_ROI_WIDTH = 832,
    RUBIK_ROI_HEIGHT = 832,
    RUBIK_MODEL_WIDTH = 320,
    RUBIK_MODEL_HEIGHT = 320,
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

static int rubik_camera_start_vpss(RubikCamera *camera, char *error, uint32_t error_len) {
    if (camera->vpss_ready) {
        return 0;
    }

    VPSS_GRP_ATTR_S group_attr;
    VPSS_CHN_ATTR_S channel_attr;
    VPSS_CROP_INFO_S crop;
    memset(&group_attr, 0, sizeof(group_attr));
    memset(&channel_attr, 0, sizeof(channel_attr));
    memset(&crop, 0, sizeof(crop));

    group_attr.u32MaxW = 1920;
    group_attr.u32MaxH = 1080;
    group_attr.enPixelFormat = PIXEL_FORMAT_NV21;
    group_attr.stFrameRate.s32SrcFrameRate = -1;
    group_attr.stFrameRate.s32DstFrameRate = -1;

    channel_attr.u32Width = RUBIK_MODEL_WIDTH;
    channel_attr.u32Height = RUBIK_MODEL_HEIGHT;
    channel_attr.enVideoFormat = VIDEO_FORMAT_LINEAR;
    channel_attr.enPixelFormat = PIXEL_FORMAT_RGB_888_PLANAR;
    channel_attr.stFrameRate.s32SrcFrameRate = -1;
    channel_attr.stFrameRate.s32DstFrameRate = -1;
    channel_attr.u32Depth = 0;
    channel_attr.bMirror = CVI_FALSE;
    channel_attr.bFlip = CVI_FALSE;
    channel_attr.stAspectRatio.enMode = ASPECT_RATIO_NONE;
    channel_attr.stNormalize.bEnable = CVI_FALSE;

    crop.bEnable = CVI_TRUE;
    crop.enCropCoordinate = VPSS_CROP_ABS_COOR;
    crop.stCropRect.s32X = RUBIK_ROI_X;
    crop.stCropRect.s32Y = RUBIK_ROI_Y;
    crop.stCropRect.u32Width = RUBIK_ROI_WIDTH;
    crop.stCropRect.u32Height = RUBIK_ROI_HEIGHT;

    CVI_S32 ret = CVI_VPSS_CreateGrp(RUBIK_VPSS_GRP, &group_attr);
    if (ret == CVI_SUCCESS) {
        ret = CVI_VPSS_SetGrpCrop(RUBIK_VPSS_GRP, &crop);
    }
    if (ret == CVI_SUCCESS) {
        ret = CVI_VPSS_SetChnAttr(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &channel_attr);
    }
    if (ret == CVI_SUCCESS) {
        ret = CVI_VPSS_EnableChn(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN);
    }
    if (ret == CVI_SUCCESS) {
        ret = CVI_VPSS_StartGrp(RUBIK_VPSS_GRP);
    }
    if (ret == CVI_SUCCESS) {
        ret = SAMPLE_COMM_VI_Bind_VPSS(0, 0, RUBIK_VPSS_GRP);
    }
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "VPSS crop/resize setup failed: %#x", ret);
        CVI_VPSS_StopGrp(RUBIK_VPSS_GRP);
        CVI_VPSS_DestroyGrp(RUBIK_VPSS_GRP);
        return -1;
    }
    camera->vpss_bound = CVI_TRUE;
    camera->vpss_ready = CVI_TRUE;
    return 0;
}

int rubik_camera_probe_vpss_frame(RubikCamera *camera, RubikCameraFrameInfo *info,
                                  char *error, uint32_t error_len) {
    if (camera == NULL || !camera->vi_ready || info == NULL) {
        set_error(error, error_len, "camera is not initialized");
        return -1;
    }
    if (rubik_camera_start_vpss(camera, error, error_len) != 0) {
        return -1;
    }

    VIDEO_FRAME_INFO_S frame;
    memset(&frame, 0, sizeof(frame));
    CVI_S32 ret = CVI_VPSS_GetChnFrame(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &frame, 3000);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "CVI_VPSS_GetChnFrame failed: %#x", ret);
        return -1;
    }
    info->width = frame.stVFrame.u32Width;
    info->height = frame.stVFrame.u32Height;
    info->pixel_format = (uint32_t)frame.stVFrame.enPixelFormat;
    for (int plane = 0; plane < 3; ++plane) {
        info->stride[plane] = frame.stVFrame.u32Stride[plane];
        info->length[plane] = frame.stVFrame.u32Length[plane];
    }

    ret = CVI_VPSS_ReleaseChnFrame(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &frame);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "CVI_VPSS_ReleaseChnFrame failed: %#x", ret);
        return -1;
    }
    return 0;
}

int rubik_camera_warmup_vpss(RubikCamera *camera, uint32_t frame_count,
                              char *error, uint32_t error_len) {
    if (camera == NULL || !camera->vi_ready) {
        set_error(error, error_len, "camera is not initialized");
        return -1;
    }
    if (rubik_camera_start_vpss(camera, error, error_len) != 0) {
        return -1;
    }

    for (uint32_t frame_index = 0; frame_index < frame_count; ++frame_index) {
        VIDEO_FRAME_INFO_S frame;
        memset(&frame, 0, sizeof(frame));
        CVI_S32 ret = CVI_VPSS_GetChnFrame(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &frame, 3000);
        if (ret != CVI_SUCCESS) {
            set_error(error, error_len, "VPSS warm-up frame %u/%u failed: %#x",
                      frame_index + 1, frame_count, ret);
            return -1;
        }
        ret = CVI_VPSS_ReleaseChnFrame(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &frame);
        if (ret != CVI_SUCCESS) {
            set_error(error, error_len, "VPSS warm-up release %u/%u failed: %#x",
                      frame_index + 1, frame_count, ret);
            return -1;
        }
    }
    return 0;
}

int rubik_camera_copy_vpss_rgb(RubikCamera *camera, uint8_t *output,
                                uint32_t output_len, RubikCameraFrameInfo *info,
                                char *error, uint32_t error_len) {
    const uint32_t plane_size = RUBIK_MODEL_WIDTH * RUBIK_MODEL_HEIGHT;
    const uint32_t required_size = plane_size * 3;
    if (camera == NULL || !camera->vi_ready || output == NULL || info == NULL) {
        set_error(error, error_len, "camera or output buffer is not initialized");
        return -1;
    }
    if (output_len < required_size) {
        set_error(error, error_len, "RGB output buffer is too small: %u < %u", output_len, required_size);
        return -1;
    }
    if (rubik_camera_start_vpss(camera, error, error_len) != 0) {
        return -1;
    }

    VIDEO_FRAME_INFO_S frame;
    memset(&frame, 0, sizeof(frame));
    CVI_S32 ret = CVI_VPSS_GetChnFrame(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &frame, 3000);
    if (ret != CVI_SUCCESS) {
        set_error(error, error_len, "CVI_VPSS_GetChnFrame failed: %#x", ret);
        return -1;
    }
    if (frame.stVFrame.u32Width != RUBIK_MODEL_WIDTH ||
        frame.stVFrame.u32Height != RUBIK_MODEL_HEIGHT ||
        frame.stVFrame.enPixelFormat != PIXEL_FORMAT_RGB_888_PLANAR) {
        set_error(error, error_len, "unexpected VPSS frame: %ux%u, pixel format=%d",
                  frame.stVFrame.u32Width, frame.stVFrame.u32Height,
                  frame.stVFrame.enPixelFormat);
        ret = CVI_FAILURE;
        goto release;
    }

    for (int plane = 0; plane < 3; ++plane) {
        const CVI_U32 stride = frame.stVFrame.u32Stride[plane];
        const CVI_U32 length = frame.stVFrame.u32Length[plane];
        if (stride < RUBIK_MODEL_WIDTH || length < stride * RUBIK_MODEL_HEIGHT) {
            set_error(error, error_len, "invalid VPSS plane %d: stride=%u length=%u", plane, stride, length);
            ret = CVI_FAILURE;
            goto release;
        }
        CVI_U8 *source = CVI_SYS_MmapCache(frame.stVFrame.u64PhyAddr[plane], length);
        if (source == CVI_NULL) {
            set_error(error, error_len, "CVI_SYS_MmapCache failed for VPSS plane %d", plane);
            ret = CVI_FAILURE;
            goto release;
        }
        CVI_SYS_IonInvalidateCache(frame.stVFrame.u64PhyAddr[plane], source, length);
        for (CVI_U32 row = 0; row < RUBIK_MODEL_HEIGHT; ++row) {
            memcpy(output + plane * plane_size + row * RUBIK_MODEL_WIDTH,
                   source + row * stride, RUBIK_MODEL_WIDTH);
        }
        CVI_SYS_Munmap(source, length);
    }

    info->width = frame.stVFrame.u32Width;
    info->height = frame.stVFrame.u32Height;
    info->pixel_format = (uint32_t)frame.stVFrame.enPixelFormat;
    for (int plane = 0; plane < 3; ++plane) {
        info->stride[plane] = frame.stVFrame.u32Stride[plane];
        info->length[plane] = frame.stVFrame.u32Length[plane];
    }

release:
    {
        CVI_S32 release_ret = CVI_VPSS_ReleaseChnFrame(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN, &frame);
        if (ret == CVI_SUCCESS && release_ret != CVI_SUCCESS) {
            set_error(error, error_len, "CVI_VPSS_ReleaseChnFrame failed: %#x", release_ret);
            ret = release_ret;
        }
    }
    return ret == CVI_SUCCESS ? 0 : -1;
}

void rubik_camera_close(RubikCamera *camera) {
    if (camera == NULL) {
        return;
    }
    if (camera->vpss_bound) {
        SAMPLE_COMM_VI_UnBind_VPSS(0, 0, RUBIK_VPSS_GRP);
    }
    if (camera->vpss_ready) {
        CVI_VPSS_DisableChn(RUBIK_VPSS_GRP, RUBIK_VPSS_CHN);
        CVI_VPSS_StopGrp(RUBIK_VPSS_GRP);
        CVI_VPSS_DestroyGrp(RUBIK_VPSS_GRP);
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
