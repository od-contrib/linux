// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote Processor Framework - device resource management
 *
 * Copyright (c) 2019 Paul Cercueil <paul@crapouillou.net>
 */

#include <linux/device.h>
#include <linux/remoteproc.h>

static void devm_rproc_remove(void *rproc)
{
	rproc_del(rproc);
}

static void devm_rproc_free(void *rproc)
{
	rproc_free(rproc);
}

/**
 * devm_rproc_add() - resource managed rproc_add()
 * @dev: the underlying device
 * @rproc: the remote processor handle to register
 *
 * This function performs like rproc_add() but the registered rproc device will
 * automatically be removed on driver detach.
 *
 * Returns 0 on success and an appropriate error code otherwise.
 */
int devm_rproc_add(struct device *dev, struct rproc *rproc)
{
	int err;

	err = rproc_add(rproc);
	if (err)
		return err;

	return devm_add_action_or_reset(dev, devm_rproc_remove, rproc);
}
EXPORT_SYMBOL(devm_rproc_add);

/**
 * devm_rproc_alloc() - resource managed rproc_alloc()
 * @dev: the underlying device
 * @name: name of this remote processor
 * @ops: platform-specific handlers (mainly start/stop)
 * @firmware: name of firmware file to load, can be NULL
 * @len: length of private data needed by the rproc driver (in bytes)
 *
 * This function performs like rproc_alloc() but the acquired rproc device will
 * automatically be released on driver detach.
 *
 * On success the new rproc is returned, and on failure, NULL.
 */
struct rproc *devm_rproc_alloc(struct device *dev, const char *name,
			       const struct rproc_ops *ops,
			       const char *firmware, int len)
{
	struct rproc *rproc;
	int err;

	rproc = rproc_alloc(dev, name, ops, firmware, len);
	if (!rproc)
		return NULL;

	err = devm_add_action_or_reset(dev, devm_rproc_free, rproc);
	if (err)
		return NULL;

	return rproc;
}
EXPORT_SYMBOL(devm_rproc_alloc);
