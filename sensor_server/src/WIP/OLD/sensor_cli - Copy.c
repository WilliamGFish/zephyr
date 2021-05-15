// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #include <string.h>
// #include <errno.h>
// #include <stdbool.h>

// #include "osi/allocator.h"

// #include "mesh_types.h"
// #include "mesh_kernel.h"
// #include "mesh_trace.h"
// #include "mesh.h"

// #include "model_op.h"
// #include "common.h"

#include "sensor_cli.h"


// #include "sdkconfig.h"

// #include "btc_ble_mesh_sensor_client.h"

/** The following are the macro definitions of sensor client
 *  model messages length, and a message is composed of three
 *  parts: Opcode + msg_value + MIC
 */
/* Sensor client messages length */
#define BT_MESH_SENSOR_DESCRIPTOR_GET_MSG_LEN (2 + 2 + 4)
#define BT_MESH_SENSOR_CADENCE_GET_MSG_LEN    (2 + 2 + 4)
#define BT_MESH_SENSOR_CADENCE_SET_MSG_LEN    /* variable */
#define BT_MESH_SENSOR_SETTINGS_GET_MSG_LEN   (2 + 2 + 4)
#define BT_MESH_SENSOR_SETTING_GET_MSG_LEN    (2 + 4 + 4)
#define BT_MESH_SENSOR_SETTING_SET_MSG_LEN    /* variable */
#define BT_MESH_SENSOR_GET_MSG_LEN            (2 + 2 + 4)
#define BT_MESH_SENSOR_COLUMN_GET_MSG_LEN     /* variable */
#define BT_MESH_SENSOR_SERIES_GET_MSG_LEN     /* variable */

static const bt_mesh_client_op_pair_t sensor_op_pair[] = {
    { BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET, BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS },
    { BT_MESH_MODEL_OP_SENSOR_CADENCE_GET,    BT_MESH_MODEL_OP_SENSOR_CADENCE_STATUS    },
    { BT_MESH_MODEL_OP_SENSOR_CADENCE_SET,    BT_MESH_MODEL_OP_SENSOR_CADENCE_STATUS    },
    { BT_MESH_MODEL_OP_SENSOR_SETTINGS_GET,   BT_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS   },
    { BT_MESH_MODEL_OP_SENSOR_SETTING_GET,    BT_MESH_MODEL_OP_SENSOR_SETTING_STATUS    },
    { BT_MESH_MODEL_OP_SENSOR_SETTING_SET,    BT_MESH_MODEL_OP_SENSOR_SETTING_STATUS    },
    { BT_MESH_MODEL_OP_SENSOR_GET,            BT_MESH_MODEL_OP_SENSOR_STATUS            },
    { BT_MESH_MODEL_OP_SENSOR_COLUMN_GET,     BT_MESH_MODEL_OP_SENSOR_COLUMN_STATUS     },
    { BT_MESH_MODEL_OP_SENSOR_SERIES_GET,     BT_MESH_MODEL_OP_SENSOR_SERIES_STATUS     },
};

static void timeout_handler(struct k_work *work)
{
    bt_mesh_sensor_client_t *client   = NULL;
    sensor_internal_data_t  *internal = NULL;
    bt_mesh_client_node_t   *node     = NULL;

    BT_WARN("Receive sensor status message timeout");

    node = CONTAINER_OF(work, bt_mesh_client_node_t, timer.work);
    if (!node || !node->ctx.model) {
        BT_ERR("%s: node parameter is NULL", __func__);
        return;
    }

    client = (bt_mesh_sensor_client_t *)node->ctx.model->user_data;
    if (!client) {
        BT_ERR("%s: model user_data is NULL", __func__);
        return;
    }

    internal = (sensor_internal_data_t *)client->internal_data;
    if (!internal) {
        BT_ERR("%s: internal_data is NULL", __func__);
        return;
    }

    bt_mesh_callback_sensor_status_to_btc(node->opcode, 0x03, node->ctx.model,
                                          &node->ctx, NULL, 0);

    sys_slist_find_and_remove(&internal->queue, &node->client_node);
    osi_free(node);

    return;
}

static void sensor_status(struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    bt_mesh_sensor_client_t *client   = NULL;
    sensor_internal_data_t  *internal = NULL;
    bt_mesh_client_node_t   *node     = NULL;
    uint8_t  *val = NULL;
    uint8_t   evt = 0xFF;
    uint32_t  rsp = 0;
    size_t len = 0;

    BT_DBG("%s: len %d, bytes %s", __func__, buf->len, bt_hex(buf->data, buf->len));

    client = (bt_mesh_sensor_client_t *)model->user_data;
    if (!client) {
        BT_ERR("%s: model user_data is NULL", __func__);
        return;
    }

    internal = (sensor_internal_data_t *)client->internal_data;
    if (!internal) {
        BT_ERR("%s: model internal_data is NULL", __func__);
        return;
    }

    rsp = ctx->recv_op;
    switch (rsp) {
    case BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS: {
        struct bt_mesh_sensor_descriptor_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_descriptor_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->descriptor = bt_mesh_alloc_buf(buf->len);
        if (!status->descriptor) {
            BT_ERR("%s: allocate memory fail", __func__);
            osi_free(status);
            return;
        }
        net_buf_simple_init(status->descriptor, 0);
        net_buf_simple_add_mem(status->descriptor, buf->data, buf->len);
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_descriptor_status);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_STATUS: {
        struct bt_mesh_sensor_cadence_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_cadence_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->property_id = net_buf_simple_pull_le16(buf);
        status->sensor_cadence_value = bt_mesh_alloc_buf(buf->len);
        if (!status->sensor_cadence_value) {
            BT_ERR("%s: allocate memory fail", __func__);
            osi_free(status);
            return;
        }
        net_buf_simple_init(status->sensor_cadence_value, 0);
        net_buf_simple_add_mem(status->sensor_cadence_value, buf->data, buf->len);
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_cadence_status);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS: {
        struct bt_mesh_sensor_settings_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_settings_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->sensor_property_id = net_buf_simple_pull_le16(buf);
        status->sensor_setting_property_ids = bt_mesh_alloc_buf(buf->len);
        if (!status->sensor_setting_property_ids) {
            BT_ERR("%s: allocate memory fail", __func__);
            osi_free(status);
            return;
        }
        net_buf_simple_init(status->sensor_setting_property_ids, 0);
        net_buf_simple_add_mem(status->sensor_setting_property_ids, buf->data, buf->len);
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_settings_status);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTING_STATUS: {
        struct bt_mesh_sensor_setting_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_setting_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->sensor_property_id         = net_buf_simple_pull_le16(buf);
        status->sensor_setting_property_id = net_buf_simple_pull_le16(buf);
        if (buf->len) {
            status->op_en                 = true;
            status->sensor_setting_access = net_buf_simple_pull_u8(buf);
            status->sensor_setting_raw = bt_mesh_alloc_buf(buf->len);
            if (!status->sensor_setting_raw) {
                BT_ERR("%s: allocate memory fail", __func__);
                osi_free(status);
                return;
            }
            net_buf_simple_init(status->sensor_setting_raw, 0);
            net_buf_simple_add_mem(status->sensor_setting_raw, buf->data, buf->len);
        }
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_setting_status);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_STATUS: {
        struct bt_mesh_sensor_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->marshalled_sensor_data = bt_mesh_alloc_buf(buf->len);
        if (!status->marshalled_sensor_data) {
            BT_ERR("%s: allocate memory fail", __func__);
            osi_free(status);
            return;
        }
        net_buf_simple_init(status->marshalled_sensor_data, 0);
        net_buf_simple_add_mem(status->marshalled_sensor_data, buf->data, buf->len);
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_status);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_COLUMN_STATUS: {
        struct bt_mesh_sensor_column_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_column_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->property_id = net_buf_simple_pull_le16(buf);
        status->sensor_column_value = bt_mesh_alloc_buf(buf->len);
        if (!status->sensor_column_value) {
            BT_ERR("%s: allocate memory fail", __func__);
            osi_free(status);
            return;
        }
        net_buf_simple_init(status->sensor_column_value, 0);
        net_buf_simple_add_mem(status->sensor_column_value, buf->data, buf->len);
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_column_status);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SERIES_STATUS: {
        struct bt_mesh_sensor_series_status *status = NULL;
        status = osi_calloc(sizeof(struct bt_mesh_sensor_series_status));
        if (!status) {
            BT_ERR("%s: allocate memory fail", __func__);
            return;
        }
        status->property_id = net_buf_simple_pull_le16(buf);
        status->sensor_series_value = bt_mesh_alloc_buf(buf->len);
        if (!status->sensor_series_value) {
            BT_ERR("%s: allocate memory fail", __func__);
            osi_free(status);
            return;
        }
        net_buf_simple_init(status->sensor_series_value, 0);
        net_buf_simple_add_mem(status->sensor_series_value, buf->data, buf->len);
        val = (uint8_t *)status;
        len = sizeof(struct bt_mesh_sensor_series_status);
        break;
    }
    default:
        BT_ERR("Not a sensor status message opcode");
        return;
    }

    buf->data = val;
    buf->len  = len;
    node = bt_mesh_is_model_message_publish(model, ctx, buf, true);
    if (!node) {
        BT_DBG("Unexpected sensor status message 0x%x", rsp);
    } else {
        switch (node->opcode) {
        case BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
        case BT_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        case BT_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        case BT_MESH_MODEL_OP_SENSOR_SETTING_GET:
        case BT_MESH_MODEL_OP_SENSOR_GET:
        case BT_MESH_MODEL_OP_SENSOR_COLUMN_GET:
        case BT_MESH_MODEL_OP_SENSOR_SERIES_GET:
            evt = 0x00;
            break;
        case BT_MESH_MODEL_OP_SENSOR_CADENCE_SET:
        case BT_MESH_MODEL_OP_SENSOR_SETTING_SET:
            evt = 0x01;
            break;
        default:
            break;
        }

        bt_mesh_callback_sensor_status_to_btc(node->opcode, evt, model, ctx, val, len);
        // Don't forget to release the node at the end.
        bt_mesh_client_free_node(&internal->queue, node);
    }

    switch (rsp) {
    case BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS: {
        struct bt_mesh_sensor_descriptor_status *status;
        status = (struct bt_mesh_sensor_descriptor_status *)val;
        bt_mesh_free_buf(status->descriptor);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_STATUS: {
        struct bt_mesh_sensor_cadence_status *status;
        status = (struct bt_mesh_sensor_cadence_status *)val;
        bt_mesh_free_buf(status->sensor_cadence_value);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS: {
        struct bt_mesh_sensor_settings_status *status;
        status = (struct bt_mesh_sensor_settings_status *)val;
        bt_mesh_free_buf(status->sensor_setting_property_ids);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTING_STATUS: {
        struct bt_mesh_sensor_setting_status *status;
        status = (struct bt_mesh_sensor_setting_status *)val;
        bt_mesh_free_buf(status->sensor_setting_raw);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_STATUS: {
        struct bt_mesh_sensor_status *status;
        status = (struct bt_mesh_sensor_status *)val;
        bt_mesh_free_buf(status->marshalled_sensor_data);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_COLUMN_STATUS: {
        struct bt_mesh_sensor_column_status *status;
        status = (struct bt_mesh_sensor_column_status *)val;
        bt_mesh_free_buf(status->sensor_column_value);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SERIES_STATUS: {
        struct bt_mesh_sensor_series_status *status;
        status = (struct bt_mesh_sensor_series_status *)val;
        bt_mesh_free_buf(status->sensor_series_value);
        break;
    }
    default:
        break;
    }

    osi_free(val);

    return;
}

const struct bt_mesh_model_op bt_mesh_sensor_cli_op[] = {
    { BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS, 0, sensor_status },
    { BT_MESH_MODEL_OP_SENSOR_CADENCE_STATUS,    2, sensor_status },
    { BT_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS,   2, sensor_status },
    { BT_MESH_MODEL_OP_SENSOR_SETTING_STATUS,    4, sensor_status },
    { BT_MESH_MODEL_OP_SENSOR_STATUS,            0, sensor_status },
    { BT_MESH_MODEL_OP_SENSOR_COLUMN_STATUS,     2, sensor_status },
    { BT_MESH_MODEL_OP_SENSOR_SERIES_STATUS,     2, sensor_status },
    BT_MESH_MODEL_OP_END,
};

static int sensor_act_state(struct bt_mesh_common_param *common,
                            void *value, uint16_t value_len, bool need_ack)
{
    struct net_buf_simple *msg = NULL;
    int err;

    msg = bt_mesh_alloc_buf(value_len);
    if (!msg) {
        BT_ERR("Sensor allocate memory fail");
        return -ENOMEM;
    }

    bt_mesh_model_msg_init(msg, common->opcode);

    switch (common->opcode) {
    case BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET: {
        struct bt_mesh_sensor_descriptor_get *act;
        act = (struct bt_mesh_sensor_descriptor_get *)value;
        if (act->op_en) {
            net_buf_simple_add_le16(msg, act->property_id);
        }
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_GET: {
        struct bt_mesh_sensor_cadence_get *act;
        act = (struct bt_mesh_sensor_cadence_get *)value;
        net_buf_simple_add_le16(msg, act->property_id);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_SET:
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK: {
        struct bt_mesh_sensor_cadence_set *act;
        act = (struct bt_mesh_sensor_cadence_set *)value;
        net_buf_simple_add_le16(msg, act->property_id);
        net_buf_simple_add_u8(msg,   act->status_trigger_type << 7 | act->fast_cadence_period_divisor);
        net_buf_simple_add_mem(msg,  act->status_trigger_delta_down->data, act->status_trigger_delta_down->len);
        net_buf_simple_add_mem(msg,  act->status_trigger_delta_up->data, act->status_trigger_delta_up->len);
        net_buf_simple_add_u8(msg,   act->status_min_interval);
        net_buf_simple_add_mem(msg,  act->fast_cadence_low->data, act->fast_cadence_low->len);
        net_buf_simple_add_mem(msg,  act->fast_cadence_high->data, act->fast_cadence_high->len);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTINGS_GET: {
        struct bt_mesh_sensor_settings_get *act;
        act = (struct bt_mesh_sensor_settings_get *)value;
        net_buf_simple_add_le16(msg, act->sensor_property_id);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTING_GET: {
        struct bt_mesh_sensor_setting_get *act;
        act = (struct bt_mesh_sensor_setting_get *)value;
        net_buf_simple_add_le16(msg, act->sensor_property_id);
        net_buf_simple_add_le16(msg, act->sensor_setting_property_id);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTING_SET:
    case BT_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK: {
        struct bt_mesh_sensor_setting_set *act;
        act = (struct bt_mesh_sensor_setting_set *)value;
        net_buf_simple_add_le16(msg, act->sensor_property_id);
        net_buf_simple_add_le16(msg, act->sensor_setting_property_id);
        net_buf_simple_add_mem(msg,  act->sensor_setting_raw->data, act->sensor_setting_raw->len);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_GET: {
        struct bt_mesh_sensor_get *act;
        act = (struct bt_mesh_sensor_get *)value;
        if (act->op_en) {
            net_buf_simple_add_le16(msg, act->property_id);
        }
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_COLUMN_GET: {
        struct bt_mesh_sensor_column_get *act;
        act = (struct bt_mesh_sensor_column_get *)value;
        net_buf_simple_add_le16(msg, act->property_id);
        net_buf_simple_add_mem(msg, act->raw_value_x->data, act->raw_value_x->len);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SERIES_GET: {
        struct bt_mesh_sensor_series_get *act;
        act = (struct bt_mesh_sensor_series_get *)value;
        net_buf_simple_add_le16(msg, act->property_id);
        if (act->op_en) {
            net_buf_simple_add_mem(msg, act->raw_value_x1->data, act->raw_value_x1->len);
            net_buf_simple_add_mem(msg, act->raw_value_x2->data, act->raw_value_x2->len);
        }
        break;
    }
    default:
        BT_ERR("Not a sensor client model message opcode");
        err = -EINVAL;
        goto end;
    }

    err = bt_mesh_client_send_msg(common->model, common->opcode, &common->ctx, msg,
                                  timeout_handler, common->msg_timeout, need_ack,
                                  common->cb, common->cb_data);
    if (err) {
        BT_ERR("Sensor message send failed (err %d)", err);
    }

end:
    bt_mesh_free_buf(msg);

    return err;
}

// int bt_mesh_sensor_client_get_state(struct bt_mesh_common_param *common, void *get, void *status)
int bt_mesh_sensor_client_get_state(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx,
			     struct net_buf_simple *buf,
                 void *get, void *status)
{
    bt_mesh_sensor_client_t *client = NULL;
    uint16_t length = 0;

    if (!model || !get) {
        BT_ERR("%s: model is NULL", __func__);
        return -EINVAL;
    }

    struct bt_mesh_sensor_cli *client = model->user_data;
    if (!client || !client->internal_data) {
        BT_ERR("%s: client parameter is NULL", __func__);
        return -EINVAL;
    }

    switch (common->opcode) {
    case BT_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
        length = BT_MESH_SENSOR_DESCRIPTOR_GET_MSG_LEN;
        break;
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_GET:
        length = BT_MESH_SENSOR_CADENCE_GET_MSG_LEN;
        break;
    case BT_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
        length = BT_MESH_SENSOR_SETTINGS_GET_MSG_LEN;
        break;
    case BT_MESH_MODEL_OP_SENSOR_SETTING_GET:
        length = BT_MESH_SENSOR_SETTING_GET_MSG_LEN;
        break;
    case BT_MESH_MODEL_OP_SENSOR_GET:
        length = BT_MESH_SENSOR_GET_MSG_LEN;
        break;
    case BT_MESH_MODEL_OP_SENSOR_COLUMN_GET: {
        struct bt_mesh_sensor_column_get *value;
        value = (struct bt_mesh_sensor_column_get *)get;
        if (!value->raw_value_x) {
            BT_ERR("Sensor column get is NULL");
            return -EINVAL;
        }
        length = (2 + 2 + value->raw_value_x->len + 4);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SERIES_GET: {
        struct bt_mesh_sensor_series_get *value;
        value = (struct bt_mesh_sensor_series_get *)get;
        if (value->op_en) {
            if (!value->raw_value_x1 || !value->raw_value_x2) {
                BT_ERR("Sensor series get parameter is NULL");
                return -EINVAL;
            }
        }
        if (value->op_en) {
            length = value->raw_value_x1->len + value->raw_value_x2->len;
        }
        length += (2 + 2 + 4);
        break;
    }
    default:
        BT_ERR("Not a sensor get message opcode");
        return -EINVAL;
    }

    return sensor_act_state(common, get, length, true);
}

int bt_mesh_sensor_client_set_state(struct bt_mesh_common_param *common, void *set, void *status)
{
    bt_mesh_sensor_client_t *client = NULL;
    uint16_t length   = 0;
    bool  need_ack = false;

    if (!common || !common->model || !set) {
        BT_ERR("%s: common parameter is NULL", __func__);
        return -EINVAL;
    }

    client = (bt_mesh_sensor_client_t *)common->model->user_data;
    if (!client || !client->internal_data) {
        BT_ERR("%s: client parameter is NULL", __func__);
        return -EINVAL;
    }

    switch (common->opcode) {
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_SET:
        need_ack = true;
    case BT_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK: {
        struct bt_mesh_sensor_cadence_set *value;
        value = (struct bt_mesh_sensor_cadence_set *)set;
        if (!value->status_trigger_delta_down || !value->status_trigger_delta_up ||
                !value->fast_cadence_low || !value->fast_cadence_high) {
            BT_ERR("Sensor cadence set parameter is NULL");
            return -EINVAL;
        }
        length = value->status_trigger_delta_down->len + \
                 value->status_trigger_delta_up->len + \
                 value->fast_cadence_low->len + \
                 value->fast_cadence_high->len;
        length += (1 + 2 + 1 + 1 + 4);
        break;
    }
    case BT_MESH_MODEL_OP_SENSOR_SETTING_SET:
        need_ack = true;
    case BT_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK: {
        struct bt_mesh_sensor_setting_set *value;
        value = (struct bt_mesh_sensor_setting_set *)set;
        if (!value->sensor_setting_raw) {
            BT_ERR("Sensor_setting_raw is NULL");
            return -EINVAL;
        }
        length = (1 + 2 + 2 + value->sensor_setting_raw->len + 4);
        break;
    }
    default:
        BT_ERR("Not a sensor set message opcode");
        return -EINVAL;
    }

    return sensor_act_state(common, set, length, need_ack);
}

int bt_mesh_sensor_cli_init(struct bt_mesh_model *model, bool primary)
{
    // bt_mesh_sensor_client_t *client   = NULL;
    // sensor_internal_data_t  *internal = NULL;

    BT_DBG("primary %u", primary);

    if (!model) {
        BT_ERR("Sensor client model is NULL");
        return -EINVAL;
    }

	if (!model->user_data) {
        BT_ERR("Sensor client model user_data is NULL");
        return -EINVAL;
	}

    client = (bt_mesh_sensor_client_t *)model->user_data;
    if (!client) {
        BT_ERR("Sensor client model user_data is NULL");
        return -EINVAL;
    }

    /* TODO: call osi_free() when deinit function is invoked*/
    internal = osi_calloc(sizeof(sensor_internal_data_t));
    if (!internal) {
        BT_ERR("Allocate memory for sensor internal data fail");
        return -ENOMEM;
    }

    sys_slist_init(&internal->queue);

    client->model         = model;
    client->op_pair_size  = ARRAY_SIZE(sensor_op_pair);
    client->op_pair       = sensor_op_pair;
    client->internal_data = internal;

    return 0;
}