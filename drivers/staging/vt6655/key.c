/*
 * Copyright (c) 1996, 2003 VIA Networking Technologies, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *
 * File: key.c
 *
 * Purpose: Implement functions for 802.11i Key management
 *
 * Author: Jerry Chen
 *
 * Date: May 29, 2003
 *
 */

#include "tmacro.h"
#include "key.h"
#include "mac.h"

/*---------------------  Static Definitions -------------------------*/

/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Functions  --------------------------*/

/*---------------------  Export Variables  --------------------------*/

/*---------------------  Static Definitions -------------------------*/

/*---------------------  Static Classes  ----------------------------*/

/*---------------------  Static Variables  --------------------------*/

/*---------------------  Static Functions  --------------------------*/
static void
s_vCheckKeyTableValid(PSKeyManagement pTable, void __iomem *dwIoBase)
{
	int i;

	for (i = 0; i < MAX_KEY_TABLE; i++) {
		if (pTable->KeyTable[i].bInUse &&
		    !pTable->KeyTable[i].PairwiseKey.bKeyValid &&
		    !pTable->KeyTable[i].GroupKey[0].bKeyValid &&
		    !pTable->KeyTable[i].GroupKey[1].bKeyValid &&
		    !pTable->KeyTable[i].GroupKey[2].bKeyValid &&
		    !pTable->KeyTable[i].GroupKey[3].bKeyValid) {
			pTable->KeyTable[i].bInUse = false;
			pTable->KeyTable[i].wKeyCtl = 0;
			pTable->KeyTable[i].bSoftWEP = false;
			MACvDisableKeyEntry(dwIoBase, i);
		}
	}
}

/*---------------------  Export Functions  --------------------------*/

/*
 * Description: Init Key management table
 *
 * Parameters:
 *  In:
 *      pTable          - Pointer to Key table
 *  Out:
 *      none
 *
 * Return Value: none
 *
 */
void KeyvInitTable(PSKeyManagement pTable, void __iomem *dwIoBase)
{
	int i;
	int jj;

	for (i = 0; i < MAX_KEY_TABLE; i++) {
		pTable->KeyTable[i].bInUse = false;
		pTable->KeyTable[i].PairwiseKey.bKeyValid = false;
		pTable->KeyTable[i].PairwiseKey.pvKeyTable = (void *)&pTable->KeyTable[i];
		for (jj = 0; jj < MAX_GROUP_KEY; jj++) {
			pTable->KeyTable[i].GroupKey[jj].bKeyValid = false;
			pTable->KeyTable[i].GroupKey[jj].pvKeyTable = (void *)&pTable->KeyTable[i];
		}
		pTable->KeyTable[i].wKeyCtl = 0;
		pTable->KeyTable[i].dwGTKeyIndex = 0;
		pTable->KeyTable[i].bSoftWEP = false;
		MACvDisableKeyEntry(dwIoBase, i);
	}
}

/*
 * Description: Get Key from table
 *
 * Parameters:
 *  In:
 *      pTable          - Pointer to Key table
 *      pbyBSSID        - BSSID of Key
 *      dwKeyIndex      - Key Index (0xFFFFFFFF means pairwise key)
 *  Out:
 *      pKey            - Key return
 *
 * Return Value: true if found otherwise false
 *
 */
bool KeybGetKey(
	PSKeyManagement pTable,
	unsigned char *pbyBSSID,
	unsigned long dwKeyIndex,
	PSKeyItem       *pKey
)
{
	int i;

	pr_debug("KeybGetKey()\n");

	*pKey = NULL;
	for (i = 0; i < MAX_KEY_TABLE; i++) {
		if (pTable->KeyTable[i].bInUse &&
		    ether_addr_equal(pTable->KeyTable[i].abyBSSID, pbyBSSID)) {
			if (dwKeyIndex == 0xFFFFFFFF) {
				if (pTable->KeyTable[i].PairwiseKey.bKeyValid) {
					*pKey = &(pTable->KeyTable[i].PairwiseKey);
					return true;
				} else {
					return false;
				}
			} else if (dwKeyIndex < MAX_GROUP_KEY) {
				if (pTable->KeyTable[i].GroupKey[dwKeyIndex].bKeyValid) {
					*pKey = &(pTable->KeyTable[i].GroupKey[dwKeyIndex]);
					return true;
				} else {
					return false;
				}
			} else {
				return false;
			}
		}
	}
	return false;
}

/*
 * Description: Set Key to table
 *
 * Parameters:
 *  In:
 *      pTable          - Pointer to Key table
 *      pbyBSSID        - BSSID of Key
 *      dwKeyIndex      - Key index (reference to NDIS DDK)
 *      uKeyLength      - Key length
 *      KeyRSC          - Key RSC
 *      pbyKey          - Pointer to key
 *  Out:
 *      none
 *
 * Return Value: true if success otherwise false
 *
 */
bool KeybSetKey(
	PSKeyManagement pTable,
	unsigned char *pbyBSSID,
	unsigned long dwKeyIndex,
	unsigned long uKeyLength,
	u64 *pKeyRSC,
	unsigned char *pbyKey,
	unsigned char byKeyDecMode,
	void __iomem *dwIoBase,
	unsigned char byLocalID
)
{
	int i, j;
	unsigned int ii;
	PSKeyItem   pKey;
	unsigned int uKeyIdx;

	pr_debug("Enter KeybSetKey: %lX\n", dwKeyIndex);

	j = (MAX_KEY_TABLE-1);
	for (i = 0; i < (MAX_KEY_TABLE - 1); i++) {
		if (!pTable->KeyTable[i].bInUse && (j == (MAX_KEY_TABLE-1))) {
			// found empty table
			j = i;
		}
		if (pTable->KeyTable[i].bInUse &&
		    ether_addr_equal(pTable->KeyTable[i].abyBSSID, pbyBSSID)) {
			// found table already exist
			if ((dwKeyIndex & PAIRWISE_KEY) != 0) {
				// Pairwise key
				pKey = &(pTable->KeyTable[i].PairwiseKey);
				pTable->KeyTable[i].wKeyCtl &= 0xFFF0;          // clear pairwise key control filed
				pTable->KeyTable[i].wKeyCtl |= byKeyDecMode;
				uKeyIdx = 4;                                    // use HW key entry 4 for pairwise key
			} else {
				// Group key
				if ((dwKeyIndex & 0x000000FF) >= MAX_GROUP_KEY)
					return false;
				pKey = &(pTable->KeyTable[i].GroupKey[dwKeyIndex & 0x000000FF]);
				if ((dwKeyIndex & TRANSMIT_KEY) != 0)  {
					// Group transmit key
					pTable->KeyTable[i].dwGTKeyIndex = dwKeyIndex;
					pr_debug("Group transmit key(R)[%lX]: %d\n",
						 pTable->KeyTable[i].dwGTKeyIndex, i);
				}
				pTable->KeyTable[i].wKeyCtl &= 0xFF0F;          // clear group key control filed
				pTable->KeyTable[i].wKeyCtl |= (byKeyDecMode << 4);
				pTable->KeyTable[i].wKeyCtl |= 0x0040;          // use group key for group address
				uKeyIdx = (dwKeyIndex & 0x000000FF);
			}
			pTable->KeyTable[i].wKeyCtl |= 0x8000;              // enable on-fly

			pKey->bKeyValid = true;
			pKey->uKeyLength = uKeyLength;
			pKey->dwKeyIndex = dwKeyIndex;
			pKey->byCipherSuite = byKeyDecMode;
			memcpy(pKey->abyKey, pbyKey, uKeyLength);
			if (byKeyDecMode == KEY_CTL_WEP) {
				if (uKeyLength == WLAN_WEP40_KEYLEN)
					pKey->abyKey[15] &= 0x7F;
				if (uKeyLength == WLAN_WEP104_KEYLEN)
					pKey->abyKey[15] |= 0x80;
			}
			MACvSetKeyEntry(dwIoBase, pTable->KeyTable[i].wKeyCtl, i, uKeyIdx, pbyBSSID, (u32 *)pKey->abyKey, byLocalID);

			if ((dwKeyIndex & USE_KEYRSC) == 0) {
				// RSC set by NIC
				pKey->KeyRSC = 0;
			} else {
				pKey->KeyRSC = *pKeyRSC;
			}
			pKey->dwTSC47_16 = 0;
			pKey->wTSC15_0 = 0;

			pr_debug("KeybSetKey(R):\n");
			pr_debug("pKey->bKeyValid: %d\n ", pKey->bKeyValid);
			pr_debug("pKey->abyKey: ");
			for (ii = 0; ii < pKey->uKeyLength; ii++)
				pr_debug("%02x ", pKey->abyKey[ii]);

			pr_debug("\n");

			pr_debug("pKey->dwTSC47_16: %lx\n ", pKey->dwTSC47_16);
			pr_debug("pKey->wTSC15_0: %x\n ", pKey->wTSC15_0);
			pr_debug("pKey->dwKeyIndex: %lx\n ", pKey->dwKeyIndex);

			return true;
		}
	}
	if (j < (MAX_KEY_TABLE-1)) {
		ether_addr_copy(pTable->KeyTable[j].abyBSSID, pbyBSSID);
		pTable->KeyTable[j].bInUse = true;
		if ((dwKeyIndex & PAIRWISE_KEY) != 0)  {
			// Pairwise key
			pKey = &(pTable->KeyTable[j].PairwiseKey);
			pTable->KeyTable[j].wKeyCtl &= 0xFFF0;          // clear pairwise key control filed
			pTable->KeyTable[j].wKeyCtl |= byKeyDecMode;
			uKeyIdx = 4;                                    // use HW key entry 4 for pairwise key
		} else {
			// Group key
			if ((dwKeyIndex & 0x000000FF) >= MAX_GROUP_KEY)
				return false;
			pKey = &(pTable->KeyTable[j].GroupKey[dwKeyIndex & 0x000000FF]);
			if ((dwKeyIndex & TRANSMIT_KEY) != 0)  {
				// Group transmit key
				pTable->KeyTable[j].dwGTKeyIndex = dwKeyIndex;
				pr_debug("Group transmit key(N)[%lX]: %d\n",
					 pTable->KeyTable[j].dwGTKeyIndex, j);
			}
			pTable->KeyTable[j].wKeyCtl &= 0xFF0F;          // clear group key control filed
			pTable->KeyTable[j].wKeyCtl |= (byKeyDecMode << 4);
			pTable->KeyTable[j].wKeyCtl |= 0x0040;          // use group key for group address
			uKeyIdx = (dwKeyIndex & 0x000000FF);
		}
		pTable->KeyTable[j].wKeyCtl |= 0x8000;              // enable on-fly

		pKey->bKeyValid = true;
		pKey->uKeyLength = uKeyLength;
		pKey->dwKeyIndex = dwKeyIndex;
		pKey->byCipherSuite = byKeyDecMode;
		memcpy(pKey->abyKey, pbyKey, uKeyLength);
		if (byKeyDecMode == KEY_CTL_WEP) {
			if (uKeyLength == WLAN_WEP40_KEYLEN)
				pKey->abyKey[15] &= 0x7F;
			if (uKeyLength == WLAN_WEP104_KEYLEN)
				pKey->abyKey[15] |= 0x80;
		}
		MACvSetKeyEntry(dwIoBase, pTable->KeyTable[j].wKeyCtl, j, uKeyIdx, pbyBSSID, (u32 *)pKey->abyKey, byLocalID);

		if ((dwKeyIndex & USE_KEYRSC) == 0) {
			// RSC set by NIC
			pKey->KeyRSC = 0;
		} else {
			pKey->KeyRSC = *pKeyRSC;
		}
		pKey->dwTSC47_16 = 0;
		pKey->wTSC15_0 = 0;

		pr_debug("KeybSetKey(N):\n");
		pr_debug("pKey->bKeyValid: %d\n ", pKey->bKeyValid);
		pr_debug("pKey->uKeyLength: %d\n ", (int)pKey->uKeyLength);
		pr_debug("pKey->abyKey: ");
		for (ii = 0; ii < pKey->uKeyLength; ii++)
			pr_debug("%02x ", pKey->abyKey[ii]);

		pr_debug("\n");

		pr_debug("pKey->dwTSC47_16: %lx\n ", pKey->dwTSC47_16);
		pr_debug("pKey->wTSC15_0: %x\n ", pKey->wTSC15_0);
		pr_debug("pKey->dwKeyIndex: %lx\n ", pKey->dwKeyIndex);

		return true;
	}
	return false;
}

/*
 * Description: Remove Key from table
 *
 * Parameters:
 *  In:
 *      pTable          - Pointer to Key table
 *      pbyBSSID        - BSSID of Key
 *      dwKeyIndex      - Key Index (reference to NDIS DDK)
 *  Out:
 *      none
 *
 * Return Value: true if success otherwise false
 *
 */
bool KeybRemoveKey(
	PSKeyManagement pTable,
	unsigned char *pbyBSSID,
	unsigned long dwKeyIndex,
	void __iomem *dwIoBase
)
{
	int  i;

	if (is_broadcast_ether_addr(pbyBSSID)) {
		// delete all keys
		if ((dwKeyIndex & PAIRWISE_KEY) != 0) {
			for (i = 0; i < MAX_KEY_TABLE; i++)
				pTable->KeyTable[i].PairwiseKey.bKeyValid = false;

			s_vCheckKeyTableValid(pTable, dwIoBase);
			return true;
		} else if ((dwKeyIndex & 0x000000FF) < MAX_GROUP_KEY) {
			for (i = 0; i < MAX_KEY_TABLE; i++) {
				pTable->KeyTable[i].GroupKey[dwKeyIndex & 0x000000FF].bKeyValid = false;
				if ((dwKeyIndex & 0x7FFFFFFF) == (pTable->KeyTable[i].dwGTKeyIndex & 0x7FFFFFFF)) {
					// remove Group transmit key
					pTable->KeyTable[i].dwGTKeyIndex = 0;
				}
			}
			s_vCheckKeyTableValid(pTable, dwIoBase);
			return true;
		}
		return false;
	}

	for (i = 0; i < MAX_KEY_TABLE; i++) {
		if (pTable->KeyTable[i].bInUse &&
		    ether_addr_equal(pTable->KeyTable[i].abyBSSID, pbyBSSID)) {
			if ((dwKeyIndex & PAIRWISE_KEY) != 0) {
				pTable->KeyTable[i].PairwiseKey.bKeyValid = false;
				s_vCheckKeyTableValid(pTable, dwIoBase);
				return true;
			} else if ((dwKeyIndex & 0x000000FF) < MAX_GROUP_KEY) {
				pTable->KeyTable[i].GroupKey[dwKeyIndex & 0x000000FF].bKeyValid = false;
				if ((dwKeyIndex & 0x7FFFFFFF) == (pTable->KeyTable[i].dwGTKeyIndex & 0x7FFFFFFF)) {
					// remove Group transmit key
					pTable->KeyTable[i].dwGTKeyIndex = 0;
				}
				s_vCheckKeyTableValid(pTable, dwIoBase);
				return true;
			}
			return false;
		}
	}
	return false;
}

/*
 * Description: Remove Key from table
 *
 * Parameters:
 *  In:
 *      pTable          - Pointer to Key table
 *      pbyBSSID        - BSSID of Key
 *  Out:
 *      none
 *
 * Return Value: true if success otherwise false
 *
 */
bool KeybRemoveAllKey(
	PSKeyManagement pTable,
	unsigned char *pbyBSSID,
	void __iomem *dwIoBase
)
{
	u32 i;

	for (i = 0; i < MAX_KEY_TABLE; i++)
		MACvDisableKeyEntry(priv->PortOffset, i);

	return 0;
}

static int vnt_set_keymode(struct ieee80211_hw *hw, u8 *mac_addr,
	struct ieee80211_key_conf *key, u32 key_type, u32 mode,
	bool onfly_latch)
{
	struct vnt_private *priv = hw->priv;
	u8 broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	u16 key_mode = 0;
	u32 entry = 0;
	u8 *bssid;
	u8 key_inx = key->keyidx;
	u8 i;

	if (mac_addr)
		bssid = mac_addr;
	else
		bssid = &broadcast[0];

	if (key_type != VNT_KEY_DEFAULTKEY) {
		for (i = 0; i < (MAX_KEY_TABLE - 1); i++) {
			if (!test_bit(i, &priv->key_entry_inuse)) {
				set_bit(i, &priv->key_entry_inuse);

				key->hw_key_idx = i;
				entry = key->hw_key_idx;
				break;
			}
		}
	}

	switch (key_type) {
	/* fallthrough */
	case VNT_KEY_DEFAULTKEY:
		/* default key last entry */
		entry = MAX_KEY_TABLE - 1;
		key->hw_key_idx = entry;
	case VNT_KEY_ALLGROUP:
		key_mode |= VNT_KEY_ALLGROUP;
		if (onfly_latch)
			key_mode |= VNT_KEY_ONFLY_ALL;
	case VNT_KEY_GROUP_ADDRESS:
		key_mode |= mode;
	case VNT_KEY_GROUP:
		key_mode |= (mode << 4);
		key_mode |= VNT_KEY_GROUP;
		break;
	case  VNT_KEY_PAIRWISE:
		key_mode |= mode;
		key_inx = 4;
		break;
	default:
		return -EINVAL;
	}

	if (onfly_latch)
		key_mode |= VNT_KEY_ONFLY;

	if (mode == KEY_CTL_WEP) {
		if (key->keylen == WLAN_KEY_LEN_WEP40)
			key->key[15] &= 0x7f;
		if (key->keylen == WLAN_KEY_LEN_WEP104)
			key->key[15] |= 0x80;
	}

	MACvSetKeyEntry(priv->PortOffset, key_mode, entry, key_inx,
			bssid, (u32 *)key->key, priv->byLocalID);

	return 0;
}

int vnt_set_keys(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
		 struct ieee80211_vif *vif, struct ieee80211_key_conf *key)
{
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct vnt_private *priv = hw->priv;
	u8 *mac_addr = NULL;
	u8 key_dec_mode = 0;
	int ret = 0;
	u32 u;

	if (sta)
		mac_addr = &sta->addr[0];

	switch (key->cipher) {
	case 0:
		for (u = 0 ; u < MAX_KEY_TABLE; u++)
			MACvDisableKeyEntry(priv->PortOffset, u);
		return ret;

	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		for (u = 0; u < MAX_KEY_TABLE; u++)
			MACvDisableKeyEntry(priv->PortOffset, u);

		vnt_set_keymode(hw, mac_addr,
				key, VNT_KEY_DEFAULTKEY, KEY_CTL_WEP, true);

		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;

		return ret;
	case WLAN_CIPHER_SUITE_TKIP:
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;

		key_dec_mode = KEY_CTL_TKIP;

		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key_dec_mode = KEY_CTL_CCMP;

		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
	}

	if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE) {
		vnt_set_keymode(hw, mac_addr,
				key, VNT_KEY_PAIRWISE, key_dec_mode, true);
	} else {
		vnt_set_keymode(hw, mac_addr,
				key, VNT_KEY_DEFAULTKEY, key_dec_mode, true);

		vnt_set_keymode(hw, (u8 *)conf->bssid,
				key, VNT_KEY_GROUP_ADDRESS, key_dec_mode, true);
	}

	return 0;
}

int vnt_key_init_table(struct vnt_private *priv)
{
	u32 i;

	for (i = 0; i < MAX_KEY_TABLE; i++)
		MACvDisableKeyEntry(priv->PortOffset, i);

	return 0;
}

static int vnt_set_keymode(struct ieee80211_hw *hw, u8 *mac_addr,
	struct ieee80211_key_conf *key, u32 key_type, u32 mode,
	bool onfly_latch)
{
	struct vnt_private *priv = hw->priv;
	u8 broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	u16 key_mode = 0;
	u32 entry = 0;
	u8 *bssid;
	u8 key_inx = key->keyidx;
	u8 i;

	if (mac_addr)
		bssid = mac_addr;
	else
		bssid = &broadcast[0];

	if (key_type != VNT_KEY_DEFAULTKEY) {
		for (i = 0; i < (MAX_KEY_TABLE - 1); i++) {
			if (!test_bit(i, &priv->key_entry_inuse)) {
				set_bit(i, &priv->key_entry_inuse);

				key->hw_key_idx = i;
				entry = key->hw_key_idx;
				break;
			}
		}
	}

	switch (key_type) {
	/* fallthrough */
	case VNT_KEY_DEFAULTKEY:
		/* default key last entry */
		entry = MAX_KEY_TABLE - 1;
		key->hw_key_idx = entry;
	case VNT_KEY_ALLGROUP:
		key_mode |= VNT_KEY_ALLGROUP;
		if (onfly_latch)
			key_mode |= VNT_KEY_ONFLY_ALL;
	case VNT_KEY_GROUP_ADDRESS:
		key_mode |= mode;
	case VNT_KEY_GROUP:
		key_mode |= (mode << 4);
		key_mode |= VNT_KEY_GROUP;
		break;
	case  VNT_KEY_PAIRWISE:
		key_mode |= mode;
		key_inx = 4;
		break;
	default:
		return -EINVAL;
	}

	if (onfly_latch)
		key_mode |= VNT_KEY_ONFLY;

	if (mode == KEY_CTL_WEP) {
		if (key->keylen == WLAN_KEY_LEN_WEP40)
			key->key[15] &= 0x7f;
		if (key->keylen == WLAN_KEY_LEN_WEP104)
			key->key[15] |= 0x80;
	}

	MACvSetKeyEntry(priv->PortOffset, key_mode, entry, key_inx,
			bssid, (u32 *)key->key, priv->byLocalID);

	return 0;
}

int vnt_set_keys(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
		 struct ieee80211_vif *vif, struct ieee80211_key_conf *key)
{
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct vnt_private *priv = hw->priv;
	u8 *mac_addr = NULL;
	u8 key_dec_mode = 0;
	int ret = 0;
	u32 u;

	if (sta)
		mac_addr = &sta->addr[0];

	switch (key->cipher) {
	case 0:
		for (u = 0 ; u < MAX_KEY_TABLE; u++)
			MACvDisableKeyEntry(priv->PortOffset, u);
		return ret;

	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		for (u = 0; u < MAX_KEY_TABLE; u++)
			MACvDisableKeyEntry(priv->PortOffset, u);

		vnt_set_keymode(hw, mac_addr,
				key, VNT_KEY_DEFAULTKEY, KEY_CTL_WEP, true);

		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;

		return ret;
	case WLAN_CIPHER_SUITE_TKIP:
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;

		key_dec_mode = KEY_CTL_TKIP;

		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key_dec_mode = KEY_CTL_CCMP;

		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
	}

	if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE) {
		vnt_set_keymode(hw, mac_addr,
				key, VNT_KEY_PAIRWISE, key_dec_mode, true);
	} else {
		vnt_set_keymode(hw, mac_addr,
				key, VNT_KEY_DEFAULTKEY, key_dec_mode, true);

		vnt_set_keymode(hw, (u8 *)conf->bssid,
				key, VNT_KEY_GROUP_ADDRESS, key_dec_mode, true);
	}

	return 0;
}
