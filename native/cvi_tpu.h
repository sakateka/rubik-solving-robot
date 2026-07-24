#ifndef RUBIK_CVI_TPU_H
#define RUBIK_CVI_TPU_H

#include <stddef.h>

typedef struct RubikCviTpu RubikCviTpu;

// All functions return 0 on success. On failure they write a short,
// NUL-terminated diagnostic into error (when error_size is non-zero).
int rubik_cvi_tpu_open(const char *model_path, RubikCviTpu **out,
                       char *error, size_t error_size);
void rubik_cvi_tpu_close(RubikCviTpu *tpu);
size_t rubik_cvi_tpu_input_len(const RubikCviTpu *tpu);
size_t rubik_cvi_tpu_output_len(const RubikCviTpu *tpu);
int rubik_cvi_tpu_forward(RubikCviTpu *tpu, const float *input,
                          size_t input_len, float *output, size_t output_len,
                          char *error, size_t error_size);

#endif
