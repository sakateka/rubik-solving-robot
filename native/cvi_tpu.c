#include "cvi_tpu.h"

#include <cviruntime.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct RubikCviTpu {
  CVI_MODEL_HANDLE model;
  CVI_TENSOR *inputs;
  CVI_TENSOR *outputs;
  int32_t input_num;
  int32_t output_num;
};

static void set_error(char *error, size_t error_size, const char *format, ...) {
  if (error == NULL || error_size == 0) return;
  va_list args;
  va_start(args, format);
  vsnprintf(error, error_size, format, args);
  va_end(args);
}

int rubik_cvi_tpu_open(const char *model_path, RubikCviTpu **out,
                       char *error, size_t error_size) {
  if (model_path == NULL || out == NULL) {
    set_error(error, error_size, "model path and output handle are required");
    return -1;
  }
  *out = NULL;
  RubikCviTpu *tpu = calloc(1, sizeof(*tpu));
  if (tpu == NULL) {
    set_error(error, error_size, "out of memory allocating TPU handle");
    return -1;
  }
  CVI_RC rc = CVI_NN_RegisterModel(model_path, &tpu->model);
  if (rc != CVI_RC_SUCCESS) {
    set_error(error, error_size, "CVI_NN_RegisterModel failed: %d", rc);
    free(tpu);
    return -1;
  }
  rc = CVI_NN_GetInputOutputTensors(tpu->model, &tpu->inputs, &tpu->input_num,
                                    &tpu->outputs, &tpu->output_num);
  if (rc != CVI_RC_SUCCESS || tpu->input_num != 1 || tpu->output_num != 1) {
    set_error(error, error_size,
              "expected one input and one output, got inputs=%d outputs=%d (rc=%d)",
              tpu->input_num, tpu->output_num, rc);
    CVI_NN_CleanupModel(tpu->model);
    free(tpu);
    return -1;
  }
  if (tpu->inputs[0].fmt != CVI_FMT_FP32 || tpu->outputs[0].fmt != CVI_FMT_FP32) {
    set_error(error, error_size, "this adapter expects FP32 external tensors");
    CVI_NN_CleanupModel(tpu->model);
    free(tpu);
    return -1;
  }
  if (CVI_NN_TensorPtr(&tpu->inputs[0]) == NULL ||
      CVI_NN_TensorPtr(&tpu->outputs[0]) == NULL) {
    set_error(error, error_size, "runtime returned a tensor without system memory");
    CVI_NN_CleanupModel(tpu->model);
    free(tpu);
    return -1;
  }
  *out = tpu;
  return 0;
}

void rubik_cvi_tpu_close(RubikCviTpu *tpu) {
  if (tpu == NULL) return;
  if (tpu->model != NULL) CVI_NN_CleanupModel(tpu->model);
  free(tpu);
}

size_t rubik_cvi_tpu_input_len(const RubikCviTpu *tpu) {
  return tpu == NULL ? 0 : CVI_NN_TensorCount(&tpu->inputs[0]);
}

size_t rubik_cvi_tpu_output_len(const RubikCviTpu *tpu) {
  return tpu == NULL ? 0 : CVI_NN_TensorCount(&tpu->outputs[0]);
}

int rubik_cvi_tpu_forward(RubikCviTpu *tpu, const float *input,
                          size_t input_len, float *output, size_t output_len,
                          char *error, size_t error_size) {
  if (tpu == NULL || input == NULL || output == NULL) {
    set_error(error, error_size, "TPU handle, input and output are required");
    return -1;
  }
  size_t expected_input = rubik_cvi_tpu_input_len(tpu);
  size_t expected_output = rubik_cvi_tpu_output_len(tpu);
  if (input_len != expected_input || output_len != expected_output) {
    set_error(error, error_size, "tensor size mismatch: input %zu/%zu, output %zu/%zu",
              input_len, expected_input, output_len, expected_output);
    return -1;
  }
  memcpy(CVI_NN_TensorPtr(&tpu->inputs[0]), input, input_len * sizeof(float));
  CVI_RC rc = CVI_NN_Forward(tpu->model, tpu->inputs, tpu->input_num,
                             tpu->outputs, tpu->output_num);
  if (rc != CVI_RC_SUCCESS) {
    set_error(error, error_size, "CVI_NN_Forward failed: %d", rc);
    return -1;
  }
  memcpy(output, CVI_NN_TensorPtr(&tpu->outputs[0]), output_len * sizeof(float));
  return 0;
}
