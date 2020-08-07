/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DRM_SYSFS_H_
#define _DRM_SYSFS_H_

struct drm_device;
struct device;

int drm_class_device_register(struct device *dev);
void drm_class_device_unregister(struct device *dev);

void drm_sysfs_hotplug_event(struct drm_device *dev);

#ifdef ASUS_ZS661KS_PROJECT
#define ASUS_NOTIFY_GHBM_ON_REQ        0
#define ASUS_NOTIFY_GHBM_ON_READY      1
#define ASUS_NOTIFY_SPOT_READY         2
#define ASUS_NOTIFY_FOD_TOUCHED        3

void asus_drm_notify(int var, int value);
#endif

#endif
