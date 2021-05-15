/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TEST_CERTS_H__
#define __TEST_CERTS_H__

static const unsigned char ca_certificate[] = {
#include "digicertMosquitto.cer"
// #include "certs.c"
};

static const unsigned char private_key[] = {
#include "digicertAzure.cer"
// #include "certs.c"
};
#endif /* __TEST_CERTS_H__ */
