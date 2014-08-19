/*
 * Linux cfg80211 driver - Dongle Host Driver (DHD) related
 *
 * Copyright (C) 2011, Broadcom Corporation
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 *
 * $Id: wl_cfg80211.c,v 1.1.4.1.2.14 2011/02/09 01:40:07 Exp $
 */


#ifndef __DHD_CFG80211__
#define __DHD_CFG80211__

#include <wl_cfg80211.h>
#include <wl_cfgp2p.h>

s32 dhd_cfg80211_init(struct wl_priv *wl);
s32 dhd_cfg80211_deinit(struct wl_priv *wl);
s32 dhd_cfg80211_get_opmode(struct wl_priv *wl);
s32 dhd_cfg80211_down(struct wl_priv *wl);
s32 dhd_cfg80211_set_p2p_info(struct wl_priv *wl, int val);
s32 dhd_cfg80211_clean_p2p_info(struct wl_priv *wl);
s32 dhd_config_dongle(struct wl_priv *wl, bool need_lock);

int wl_cfg80211_btcoex_init(struct wl_priv *wl);
void wl_cfg80211_btcoex_deinit(struct wl_priv *wl);

#endif /* __DHD_CFG80211__ */
