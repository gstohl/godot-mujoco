#ifndef GODOT_MUJOCO_GMJ_BRIDGE_H_
#define GODOT_MUJOCO_GMJ_BRIDGE_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gmj_model gmj_model;
typedef struct gmj_data gmj_data;

typedef enum gmj_error_code {
  GMJ_OK = 0,
  GMJ_ERR_INVALID_ARGUMENT = 1,
  GMJ_ERR_LOAD_MODEL = 2,
  GMJ_ERR_ALLOCATION = 3,
  GMJ_ERR_INDEX_OUT_OF_RANGE = 4,
  GMJ_ERR_MUJOCO = 5
} gmj_error_code;

const char* gmj_mujoco_version(void);

gmj_model* gmj_model_load_xml(const char* xml_path, char* error_buffer,
                              size_t error_buffer_size);
void gmj_model_free(gmj_model* model);

gmj_data* gmj_data_create(const gmj_model* model);
void gmj_data_free(gmj_data* data);

gmj_error_code gmj_reset_data(const gmj_model* model, gmj_data* data);
gmj_error_code gmj_step(const gmj_model* model, gmj_data* data, int steps);

int gmj_nq(const gmj_model* model);
int gmj_nv(const gmj_model* model);
int gmj_nu(const gmj_model* model);
int gmj_nbody(const gmj_model* model);

gmj_error_code gmj_set_ctrl(const gmj_model* model, gmj_data* data,
                            int actuator_index, double value);
gmj_error_code gmj_get_ctrl(const gmj_model* model, const gmj_data* data,
                            int actuator_index, double* out_value);

gmj_error_code gmj_get_qpos(const gmj_model* model, const gmj_data* data,
                            int qpos_index, double* out_value);
gmj_error_code gmj_set_qpos(const gmj_model* model, gmj_data* data,
                            int qpos_index, double value);

gmj_error_code gmj_get_qvel(const gmj_model* model, const gmj_data* data,
                            int qvel_index, double* out_value);
gmj_error_code gmj_set_qvel(const gmj_model* model, gmj_data* data,
                            int qvel_index, double value);

gmj_error_code gmj_body_world_position(const gmj_model* model,
                                       const gmj_data* data, int body_index,
                                       double* out_xyz_3);

gmj_error_code gmj_forward(const gmj_model* model, gmj_data* data);

const char* gmj_last_mujoco_error(void);

#ifdef __cplusplus
}
#endif

#endif
