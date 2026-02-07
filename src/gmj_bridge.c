#include "../include/godot_mujoco/gmj_bridge.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__has_include)
#if __has_include(<mujoco/mujoco.h>)
#include <mujoco/mujoco.h>
#define GMJ_HAS_MUJOCO 1
#else
#define GMJ_HAS_MUJOCO 0
#endif
#else
#include <mujoco/mujoco.h>
#define GMJ_HAS_MUJOCO 1
#endif

#if GMJ_HAS_MUJOCO
struct gmj_model {
  mjModel* handle;
};

struct gmj_data {
  mjData* handle;
};
#else
struct gmj_model {
  void* handle;
};

struct gmj_data {
  void* handle;
};
#endif

static _Thread_local char gmj_error_storage[1024];

static void gmj_set_error(const char* message) {
  if (message == NULL) {
    gmj_error_storage[0] = '\0';
    return;
  }

  strncpy(gmj_error_storage, message, sizeof(gmj_error_storage));
  gmj_error_storage[sizeof(gmj_error_storage) - 1] = '\0';
}

#if GMJ_HAS_MUJOCO
static gmj_error_code gmj_validate_ptrs(const gmj_model* model,
                                        const gmj_data* data) {
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  return GMJ_OK;
}

static gmj_error_code gmj_validate_slice(int start_index, int count,
                                         int length) {
  if (count < 0 || start_index < 0) {
    gmj_set_error("slice start/count must be non-negative");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (start_index > length || count > (length - start_index)) {
    gmj_set_error("slice out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }
  return GMJ_OK;
}

const char* gmj_mujoco_version(void) {
  static _Thread_local char version[32];
  const int ver = mj_version();
  const int major = ver / 100;
  const int minor = ver % 100;
  snprintf(version, sizeof(version), "%d.%d", major, minor);
  return version;
}

gmj_model* gmj_model_load_xml(const char* xml_path, char* error_buffer,
                              size_t error_buffer_size) {
  char load_error[1024] = {0};
  gmj_model* wrapper = NULL;
  mjModel* model = NULL;

  if (xml_path == NULL) {
    const char* message = "xml_path is null";
    if (error_buffer != NULL && error_buffer_size > 0) {
      strncpy(error_buffer, message, error_buffer_size);
      error_buffer[error_buffer_size - 1] = '\0';
    }
    gmj_set_error(message);
    return NULL;
  }

  model = mj_loadXML(xml_path, NULL, load_error, sizeof(load_error));
  if (model == NULL) {
    if (error_buffer != NULL && error_buffer_size > 0) {
      strncpy(error_buffer, load_error, error_buffer_size);
      error_buffer[error_buffer_size - 1] = '\0';
    }
    gmj_set_error(load_error);
    return NULL;
  }

  wrapper = (gmj_model*)malloc(sizeof(gmj_model));
  if (wrapper == NULL) {
    mj_deleteModel(model);
    gmj_set_error("failed to allocate gmj_model");
    return NULL;
  }

  wrapper->handle = model;
  gmj_set_error(NULL);
  return wrapper;
}

void gmj_model_free(gmj_model* model) {
  if (model == NULL) {
    return;
  }
  if (model->handle != NULL) {
    mj_deleteModel(model->handle);
    model->handle = NULL;
  }
  free(model);
}

gmj_data* gmj_data_create(const gmj_model* model) {
  gmj_data* wrapper = NULL;
  mjData* data = NULL;

  if (model == NULL || model->handle == NULL) {
    gmj_set_error("model is null");
    return NULL;
  }

  data = mj_makeData(model->handle);
  if (data == NULL) {
    gmj_set_error("failed to allocate mjData");
    return NULL;
  }

  wrapper = (gmj_data*)malloc(sizeof(gmj_data));
  if (wrapper == NULL) {
    mj_deleteData(data);
    gmj_set_error("failed to allocate gmj_data");
    return NULL;
  }

  wrapper->handle = data;
  gmj_set_error(NULL);
  return wrapper;
}

void gmj_data_free(gmj_data* data) {
  if (data == NULL) {
    return;
  }
  if (data->handle != NULL) {
    mj_deleteData(data->handle);
    data->handle = NULL;
  }
  free(data);
}

gmj_error_code gmj_reset_data(const gmj_model* model, gmj_data* data) {
  const gmj_error_code valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }

  mj_resetData(model->handle, data->handle);
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_step(const gmj_model* model, gmj_data* data, int steps) {
  int i = 0;
  const gmj_error_code valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (steps < 1) {
    gmj_set_error("steps must be >= 1");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  for (i = 0; i < steps; ++i) {
    mj_step(model->handle, data->handle);
  }

  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_forward(const gmj_model* model, gmj_data* data) {
  const gmj_error_code valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }

  mj_forward(model->handle, data->handle);
  gmj_set_error(NULL);
  return GMJ_OK;
}

int gmj_nq(const gmj_model* model) {
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("model is null");
    return -1;
  }
  return model->handle->nq;
}

int gmj_nv(const gmj_model* model) {
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("model is null");
    return -1;
  }
  return model->handle->nv;
}

int gmj_nu(const gmj_model* model) {
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("model is null");
    return -1;
  }
  return model->handle->nu;
}

int gmj_nbody(const gmj_model* model) {
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("model is null");
    return -1;
  }
  return model->handle->nbody;
}

int gmj_body_id(const gmj_model* model, const char* body_name) {
  int id = -1;
  if (model == NULL || model->handle == NULL || body_name == NULL) {
    gmj_set_error("invalid model pointer or body_name");
    return -1;
  }

  id = mj_name2id(model->handle, mjOBJ_BODY, body_name);
  if (id < 0) {
    gmj_set_error("body_name not found");
    return -1;
  }
  gmj_set_error(NULL);
  return id;
}

int gmj_joint_id(const gmj_model* model, const char* joint_name) {
  int id = -1;
  if (model == NULL || model->handle == NULL || joint_name == NULL) {
    gmj_set_error("invalid model pointer or joint_name");
    return -1;
  }

  id = mj_name2id(model->handle, mjOBJ_JOINT, joint_name);
  if (id < 0) {
    gmj_set_error("joint_name not found");
    return -1;
  }
  gmj_set_error(NULL);
  return id;
}

int gmj_actuator_id(const gmj_model* model, const char* actuator_name) {
  int id = -1;
  if (model == NULL || model->handle == NULL || actuator_name == NULL) {
    gmj_set_error("invalid model pointer or actuator_name");
    return -1;
  }

  id = mj_name2id(model->handle, mjOBJ_ACTUATOR, actuator_name);
  if (id < 0) {
    gmj_set_error("actuator_name not found");
    return -1;
  }
  gmj_set_error(NULL);
  return id;
}

const char* gmj_body_name(const gmj_model* model, int body_id) {
  const char* name = NULL;
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("invalid model pointer");
    return NULL;
  }
  if (body_id < 0 || body_id >= model->handle->nbody) {
    gmj_set_error("body_id out of range");
    return NULL;
  }

  name = mj_id2name(model->handle, mjOBJ_BODY, body_id);
  gmj_set_error(NULL);
  return name;
}

const char* gmj_joint_name(const gmj_model* model, int joint_id) {
  const char* name = NULL;
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("invalid model pointer");
    return NULL;
  }
  if (joint_id < 0 || joint_id >= model->handle->njnt) {
    gmj_set_error("joint_id out of range");
    return NULL;
  }

  name = mj_id2name(model->handle, mjOBJ_JOINT, joint_id);
  gmj_set_error(NULL);
  return name;
}

const char* gmj_actuator_name(const gmj_model* model, int actuator_id) {
  const char* name = NULL;
  if (model == NULL || model->handle == NULL) {
    gmj_set_error("invalid model pointer");
    return NULL;
  }
  if (actuator_id < 0 || actuator_id >= model->handle->nu) {
    gmj_set_error("actuator_id out of range");
    return NULL;
  }

  name = mj_id2name(model->handle, mjOBJ_ACTUATOR, actuator_id);
  gmj_set_error(NULL);
  return name;
}

gmj_error_code gmj_set_ctrl(const gmj_model* model, gmj_data* data,
                            int actuator_index, double value) {
  const gmj_error_code valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (actuator_index < 0 || actuator_index >= model->handle->nu) {
    gmj_set_error("actuator_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  data->handle->ctrl[actuator_index] = (mjtNum)value;
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_get_ctrl(const gmj_model* model, const gmj_data* data,
                            int actuator_index, double* out_value) {
  if (out_value == NULL) {
    gmj_set_error("out_value is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (actuator_index < 0 || actuator_index >= model->handle->nu) {
    gmj_set_error("actuator_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  *out_value = (double)data->handle->ctrl[actuator_index];
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_get_qpos(const gmj_model* model, const gmj_data* data,
                            int qpos_index, double* out_value) {
  if (out_value == NULL) {
    gmj_set_error("out_value is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (qpos_index < 0 || qpos_index >= model->handle->nq) {
    gmj_set_error("qpos_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  *out_value = (double)data->handle->qpos[qpos_index];
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_set_qpos(const gmj_model* model, gmj_data* data,
                            int qpos_index, double value) {
  const gmj_error_code valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (qpos_index < 0 || qpos_index >= model->handle->nq) {
    gmj_set_error("qpos_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  data->handle->qpos[qpos_index] = (mjtNum)value;
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_get_qvel(const gmj_model* model, const gmj_data* data,
                            int qvel_index, double* out_value) {
  if (out_value == NULL) {
    gmj_set_error("out_value is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (qvel_index < 0 || qvel_index >= model->handle->nv) {
    gmj_set_error("qvel_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  *out_value = (double)data->handle->qvel[qvel_index];
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_set_qvel(const gmj_model* model, gmj_data* data,
                            int qvel_index, double value) {
  const gmj_error_code valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (qvel_index < 0 || qvel_index >= model->handle->nv) {
    gmj_set_error("qvel_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  data->handle->qvel[qvel_index] = (mjtNum)value;
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_get_qpos_slice(const gmj_model* model, const gmj_data* data,
                                  int start_index, int count,
                                  double* out_values) {
  int i = 0;
  gmj_error_code valid = GMJ_OK;
  if (out_values == NULL) {
    gmj_set_error("out_values is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  valid = gmj_validate_slice(start_index, count, model->handle->nq);
  if (valid != GMJ_OK) {
    return valid;
  }

  for (i = 0; i < count; ++i) {
    out_values[i] = (double)data->handle->qpos[start_index + i];
  }
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_set_qpos_slice(const gmj_model* model, gmj_data* data,
                                  int start_index, int count,
                                  const double* values) {
  int i = 0;
  gmj_error_code valid = GMJ_OK;
  valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (values == NULL) {
    gmj_set_error("values is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  valid = gmj_validate_slice(start_index, count, model->handle->nq);
  if (valid != GMJ_OK) {
    return valid;
  }

  for (i = 0; i < count; ++i) {
    data->handle->qpos[start_index + i] = (mjtNum)values[i];
  }
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_get_qvel_slice(const gmj_model* model, const gmj_data* data,
                                  int start_index, int count,
                                  double* out_values) {
  int i = 0;
  gmj_error_code valid = GMJ_OK;
  if (out_values == NULL) {
    gmj_set_error("out_values is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  valid = gmj_validate_slice(start_index, count, model->handle->nv);
  if (valid != GMJ_OK) {
    return valid;
  }

  for (i = 0; i < count; ++i) {
    out_values[i] = (double)data->handle->qvel[start_index + i];
  }
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_set_qvel_slice(const gmj_model* model, gmj_data* data,
                                  int start_index, int count,
                                  const double* values) {
  int i = 0;
  gmj_error_code valid = GMJ_OK;
  valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (values == NULL) {
    gmj_set_error("values is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  valid = gmj_validate_slice(start_index, count, model->handle->nv);
  if (valid != GMJ_OK) {
    return valid;
  }

  for (i = 0; i < count; ++i) {
    data->handle->qvel[start_index + i] = (mjtNum)values[i];
  }
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_get_ctrl_slice(const gmj_model* model, const gmj_data* data,
                                  int start_index, int count,
                                  double* out_values) {
  int i = 0;
  gmj_error_code valid = GMJ_OK;
  if (out_values == NULL) {
    gmj_set_error("out_values is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  valid = gmj_validate_slice(start_index, count, model->handle->nu);
  if (valid != GMJ_OK) {
    return valid;
  }

  for (i = 0; i < count; ++i) {
    out_values[i] = (double)data->handle->ctrl[start_index + i];
  }
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_set_ctrl_slice(const gmj_model* model, gmj_data* data,
                                  int start_index, int count,
                                  const double* values) {
  int i = 0;
  gmj_error_code valid = GMJ_OK;
  valid = gmj_validate_ptrs(model, data);
  if (valid != GMJ_OK) {
    return valid;
  }
  if (values == NULL) {
    gmj_set_error("values is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }

  valid = gmj_validate_slice(start_index, count, model->handle->nu);
  if (valid != GMJ_OK) {
    return valid;
  }

  for (i = 0; i < count; ++i) {
    data->handle->ctrl[start_index + i] = (mjtNum)values[i];
  }
  gmj_set_error(NULL);
  return GMJ_OK;
}

gmj_error_code gmj_body_world_position(const gmj_model* model,
                                       const gmj_data* data, int body_index,
                                       double* out_xyz_3) {
  const mjtNum* body_xpos = NULL;
  if (out_xyz_3 == NULL) {
    gmj_set_error("out_xyz_3 is null");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (model == NULL || model->handle == NULL || data == NULL ||
      data->handle == NULL) {
    gmj_set_error("invalid model or data pointer");
    return GMJ_ERR_INVALID_ARGUMENT;
  }
  if (body_index < 0 || body_index >= model->handle->nbody) {
    gmj_set_error("body_index out of range");
    return GMJ_ERR_INDEX_OUT_OF_RANGE;
  }

  body_xpos = data->handle->xpos + (3 * body_index);
  out_xyz_3[0] = (double)body_xpos[0];
  out_xyz_3[1] = (double)body_xpos[1];
  out_xyz_3[2] = (double)body_xpos[2];
  gmj_set_error(NULL);
  return GMJ_OK;
}

const char* gmj_last_mujoco_error(void) {
  return gmj_error_storage;
}

#else

static gmj_error_code gmj_unavailable(void) {
  gmj_set_error("MuJoCo headers unavailable at build time");
  return GMJ_ERR_MUJOCO;
}

const char* gmj_mujoco_version(void) { return "unavailable"; }

gmj_model* gmj_model_load_xml(const char* xml_path, char* error_buffer,
                              size_t error_buffer_size) {
  (void)xml_path;
  if (error_buffer != NULL && error_buffer_size > 0) {
    const char* message = "MuJoCo headers unavailable at build time";
    strncpy(error_buffer, message, error_buffer_size);
    error_buffer[error_buffer_size - 1] = '\0';
  }
  gmj_unavailable();
  return NULL;
}

void gmj_model_free(gmj_model* model) { (void)model; }

gmj_data* gmj_data_create(const gmj_model* model) {
  (void)model;
  gmj_unavailable();
  return NULL;
}

void gmj_data_free(gmj_data* data) { (void)data; }

gmj_error_code gmj_reset_data(const gmj_model* model, gmj_data* data) {
  (void)model;
  (void)data;
  return gmj_unavailable();
}

gmj_error_code gmj_step(const gmj_model* model, gmj_data* data, int steps) {
  (void)model;
  (void)data;
  (void)steps;
  return gmj_unavailable();
}

int gmj_nq(const gmj_model* model) {
  (void)model;
  gmj_unavailable();
  return -1;
}

int gmj_nv(const gmj_model* model) {
  (void)model;
  gmj_unavailable();
  return -1;
}

int gmj_nu(const gmj_model* model) {
  (void)model;
  gmj_unavailable();
  return -1;
}

int gmj_nbody(const gmj_model* model) {
  (void)model;
  gmj_unavailable();
  return -1;
}

int gmj_body_id(const gmj_model* model, const char* body_name) {
  (void)model;
  (void)body_name;
  gmj_unavailable();
  return -1;
}

int gmj_joint_id(const gmj_model* model, const char* joint_name) {
  (void)model;
  (void)joint_name;
  gmj_unavailable();
  return -1;
}

int gmj_actuator_id(const gmj_model* model, const char* actuator_name) {
  (void)model;
  (void)actuator_name;
  gmj_unavailable();
  return -1;
}

const char* gmj_body_name(const gmj_model* model, int body_id) {
  (void)model;
  (void)body_id;
  gmj_unavailable();
  return NULL;
}

const char* gmj_joint_name(const gmj_model* model, int joint_id) {
  (void)model;
  (void)joint_id;
  gmj_unavailable();
  return NULL;
}

const char* gmj_actuator_name(const gmj_model* model, int actuator_id) {
  (void)model;
  (void)actuator_id;
  gmj_unavailable();
  return NULL;
}

gmj_error_code gmj_set_ctrl(const gmj_model* model, gmj_data* data,
                            int actuator_index, double value) {
  (void)model;
  (void)data;
  (void)actuator_index;
  (void)value;
  return gmj_unavailable();
}

gmj_error_code gmj_get_ctrl(const gmj_model* model, const gmj_data* data,
                            int actuator_index, double* out_value) {
  (void)model;
  (void)data;
  (void)actuator_index;
  (void)out_value;
  return gmj_unavailable();
}

gmj_error_code gmj_get_qpos(const gmj_model* model, const gmj_data* data,
                            int qpos_index, double* out_value) {
  (void)model;
  (void)data;
  (void)qpos_index;
  (void)out_value;
  return gmj_unavailable();
}

gmj_error_code gmj_set_qpos(const gmj_model* model, gmj_data* data,
                            int qpos_index, double value) {
  (void)model;
  (void)data;
  (void)qpos_index;
  (void)value;
  return gmj_unavailable();
}

gmj_error_code gmj_get_qvel(const gmj_model* model, const gmj_data* data,
                            int qvel_index, double* out_value) {
  (void)model;
  (void)data;
  (void)qvel_index;
  (void)out_value;
  return gmj_unavailable();
}

gmj_error_code gmj_set_qvel(const gmj_model* model, gmj_data* data,
                            int qvel_index, double value) {
  (void)model;
  (void)data;
  (void)qvel_index;
  (void)value;
  return gmj_unavailable();
}

gmj_error_code gmj_get_qpos_slice(const gmj_model* model, const gmj_data* data,
                                  int start_index, int count,
                                  double* out_values) {
  (void)model;
  (void)data;
  (void)start_index;
  (void)count;
  (void)out_values;
  return gmj_unavailable();
}

gmj_error_code gmj_set_qpos_slice(const gmj_model* model, gmj_data* data,
                                  int start_index, int count,
                                  const double* values) {
  (void)model;
  (void)data;
  (void)start_index;
  (void)count;
  (void)values;
  return gmj_unavailable();
}

gmj_error_code gmj_get_qvel_slice(const gmj_model* model, const gmj_data* data,
                                  int start_index, int count,
                                  double* out_values) {
  (void)model;
  (void)data;
  (void)start_index;
  (void)count;
  (void)out_values;
  return gmj_unavailable();
}

gmj_error_code gmj_set_qvel_slice(const gmj_model* model, gmj_data* data,
                                  int start_index, int count,
                                  const double* values) {
  (void)model;
  (void)data;
  (void)start_index;
  (void)count;
  (void)values;
  return gmj_unavailable();
}

gmj_error_code gmj_get_ctrl_slice(const gmj_model* model, const gmj_data* data,
                                  int start_index, int count,
                                  double* out_values) {
  (void)model;
  (void)data;
  (void)start_index;
  (void)count;
  (void)out_values;
  return gmj_unavailable();
}

gmj_error_code gmj_set_ctrl_slice(const gmj_model* model, gmj_data* data,
                                  int start_index, int count,
                                  const double* values) {
  (void)model;
  (void)data;
  (void)start_index;
  (void)count;
  (void)values;
  return gmj_unavailable();
}

gmj_error_code gmj_body_world_position(const gmj_model* model,
                                       const gmj_data* data, int body_index,
                                       double* out_xyz_3) {
  (void)model;
  (void)data;
  (void)body_index;
  (void)out_xyz_3;
  return gmj_unavailable();
}

gmj_error_code gmj_forward(const gmj_model* model, gmj_data* data) {
  (void)model;
  (void)data;
  return gmj_unavailable();
}

const char* gmj_last_mujoco_error(void) { return gmj_error_storage; }

#endif
