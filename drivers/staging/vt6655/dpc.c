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
 * File: dpc.c
 *
 * Purpose: handle dpc rx functions
 *
 * Author: Lyndon Chen
 *
 * Date: May 20, 2003
 *
 * Functions:
 *
 * Revision History:
 *
 */

#include "device.h"
#include "baseband.h"
#include "rf.h"
#include "dpc.h"

static bool vnt_rx_data(struct vnt_private *priv, struct sk_buff *skb,
			u16 bytes_received)
{
	struct ieee80211_hw *hw = priv->hw;
	struct ieee80211_supported_band *sband;
	struct ieee80211_rx_status rx_status = { 0 };
	struct ieee80211_hdr *hdr;
	__le16 fc;
	u8 *rsr, *new_rsr, *rssi;
	__le64 *tsf_time;
	u16 frame_size;
	int ii, r;
	u8 *rx_sts, *rx_rate, *sq;
	u8 *skb_data;
	u8 rate_idx = 0;
	u8 rate[MAX_RATE] = {2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108};
	long rx_dbm;

	/* [31:16]RcvByteCount ( not include 4-byte Status ) */
	frame_size = le16_to_cpu(*((__le16 *)(skb->data + 2)));
	if (frame_size > 2346 || frame_size < 14) {
		dev_dbg(&priv->pcid->dev, "------- WRONG Length 1\n");
		return false;
	}

	skb_data = (u8 *)skb->data;

	rx_sts = skb_data;
	rx_rate = skb_data + 1;

	sband = hw->wiphy->bands[hw->conf.chandef.chan->band];

	for (r = RATE_1M; r < MAX_RATE; r++) {
		if (*rx_rate == rate[r])
			break;
	}

	if ((pKey != NULL) && (pKey->byCipherSuite == KEY_CTL_TKIP)) {
		if (bIsWEP)
			FrameSize -= 8;  //MIC
	}

	//--------------------------------------------------------------------------------
	// Soft MIC
	if ((pKey != NULL) && (pKey->byCipherSuite == KEY_CTL_TKIP)) {
		if (bIsWEP) {
			__le32 *pdwMIC_L;
			__le32 *pdwMIC_R;
			__le32 dwMIC_Priority;
			__le32 dwMICKey0 = 0, dwMICKey1 = 0;
			u32 dwLocalMIC_L = 0;
			u32 dwLocalMIC_R = 0;
			viawget_wpa_header *wpahdr;

			if (pMgmt->eCurrMode == WMAC_MODE_ESS_AP) {
				dwMICKey0 = cpu_to_le32(*(u32 *)(&pKey->abyKey[24]));
				dwMICKey1 = cpu_to_le32(*(u32 *)(&pKey->abyKey[28]));
			} else {
				if (pDevice->pMgmt->eAuthenMode == WMAC_AUTH_WPANONE) {
					dwMICKey0 = cpu_to_le32(*(u32 *)(&pKey->abyKey[16]));
					dwMICKey1 = cpu_to_le32(*(u32 *)(&pKey->abyKey[20]));
				} else if ((pKey->dwKeyIndex & BIT28) == 0) {
					dwMICKey0 = cpu_to_le32(*(u32 *)(&pKey->abyKey[16]));
					dwMICKey1 = cpu_to_le32(*(u32 *)(&pKey->abyKey[20]));
				} else {
					dwMICKey0 = cpu_to_le32(*(u32 *)(&pKey->abyKey[24]));
					dwMICKey1 = cpu_to_le32(*(u32 *)(&pKey->abyKey[28]));
				}
			}

			MIC_vInit(dwMICKey0, dwMICKey1);
			MIC_vAppend((unsigned char *)&(pDevice->sRxEthHeader.abyDstAddr[0]), 12);
			dwMIC_Priority = 0;
			MIC_vAppend((unsigned char *)&dwMIC_Priority, 4);
			// 4 is Rcv buffer header, 24 is MAC Header, and 8 is IV and Ext IV.
			MIC_vAppend((unsigned char *)(skb->data + 4 + WLAN_HDR_ADDR3_LEN + 8),
				    FrameSize - WLAN_HDR_ADDR3_LEN - 8);
			MIC_vGetMIC(&dwLocalMIC_L, &dwLocalMIC_R);
			MIC_vUnInit();

			pdwMIC_L = (__le32 *)(skb->data + 4 + FrameSize);
			pdwMIC_R = (__le32 *)(skb->data + 4 + FrameSize + 4);

			if ((le32_to_cpu(*pdwMIC_L) != dwLocalMIC_L) ||
			    (le32_to_cpu(*pdwMIC_R) != dwLocalMIC_R) ||
			    pDevice->bRxMICFail) {
				pr_debug("MIC comparison is fail!\n");
				pDevice->bRxMICFail = false;
				pDevice->s802_11Counter.TKIPLocalMICFailures++;
				if (bDeFragRx) {
					if (!device_alloc_frag_buf(pDevice, &pDevice->sRxDFCB[pDevice->uCurrentDFCBIdx])) {
						pr_err("%s: can not alloc more frag bufs\n",
						       pDevice->dev->name);
					}
				}
				//2008-0409-07, <Add> by Einsn Liu
#ifdef WPA_SUPPLICANT_DRIVER_WEXT_SUPPORT
				//send event to wpa_supplicant
				{
					union iwreq_data wrqu;
					struct iw_michaelmicfailure ev;
					int keyidx = pbyFrame[cbHeaderSize+3] >> 6; //top two-bits

					memset(&ev, 0, sizeof(ev));
					ev.flags = keyidx & IW_MICFAILURE_KEY_ID;
					if ((pMgmt->eCurrMode == WMAC_MODE_ESS_STA) &&
					    (pMgmt->eCurrState == WMAC_STATE_ASSOC) &&
					    (*pbyRsr & (RSR_ADDRBROAD | RSR_ADDRMULTI)) == 0) {
						ev.flags |= IW_MICFAILURE_PAIRWISE;
					} else {
						ev.flags |= IW_MICFAILURE_GROUP;
					}

					ev.src_addr.sa_family = ARPHRD_ETHER;
					ether_addr_copy(ev.src_addr.sa_data,
							pMACHeader->abyAddr2);
					memset(&wrqu, 0, sizeof(wrqu));
					wrqu.data.length = sizeof(ev);
					wireless_send_event(pDevice->dev, IWEVMICHAELMICFAILURE, &wrqu, (char *)&ev);

				}
#endif

	for (ii = 0; ii < sband->n_bitrates; ii++) {
		if (sband->bitrates[ii].hw_value == r) {
			rate_idx = ii;
				break;
		}
	}

	if (ii == sband->n_bitrates) {
		dev_dbg(&priv->pcid->dev, "Wrong RxRate %x\n", *rx_rate);
		return false;
	}

	tsf_time = (__le64 *)(skb_data + bytes_received - 12);
	sq = skb_data + bytes_received - 4;
	new_rsr = skb_data + bytes_received - 3;
	rssi = skb_data + bytes_received - 2;
	rsr = skb_data + bytes_received - 1;
	if (*rsr & (RSR_IVLDTYP | RSR_IVLDLEN))
		return false;

	RFvRSSITodBm(priv, *rssi, &rx_dbm);

	priv->byBBPreEDRSSI = (u8)rx_dbm + 1;
	priv->uCurrRSSI = *rssi;

	skb_pull(skb, 4);
	skb_trim(skb, frame_size);

	rx_status.mactime = le64_to_cpu(*tsf_time);
	rx_status.band = hw->conf.chandef.chan->band;
	rx_status.signal = rx_dbm;
	rx_status.flag = 0;
	rx_status.freq = hw->conf.chandef.chan->center_freq;

	if (!(*rsr & RSR_CRCOK))
		rx_status.flag |= RX_FLAG_FAILED_FCS_CRC;

	hdr = (struct ieee80211_hdr *)(skb->data);
	fc = hdr->frame_control;

	rx_status.rate_idx = rate_idx;

	if (ieee80211_has_protected(fc)) {
		if (priv->byLocalID > REV_ID_VT3253_A1)
			rx_status.flag |= RX_FLAG_DECRYPTED;

		/* Drop packet */
		if (!(*new_rsr & NEWRSR_DECRYPTOK))
			return false;
	}

	memcpy(IEEE80211_SKB_RXCB(skb), &rx_status, sizeof(rx_status));

	ieee80211_rx_irqsafe(priv->hw, skb);

	return true;
}

bool vnt_receive_frame(struct vnt_private *priv, PSRxDesc curr_rd)
{
	PDEVICE_RD_INFO rd_info = curr_rd->pRDInfo;
	struct sk_buff *skb;
	u16 frame_size;

	skb = rd_info->skb;

	pci_unmap_single(priv->pcid, rd_info->skb_dma,
			 priv->rx_buf_sz, PCI_DMA_FROMDEVICE);

	frame_size = le16_to_cpu(curr_rd->m_rd1RD1.wReqCount)
			- cpu_to_le16(curr_rd->m_rd0RD0.wResCount);

	if ((frame_size > 2364) || (frame_size < 33)) {
		/* Frame Size error drop this packet.*/
		dev_dbg(&priv->pcid->dev, "Wrong frame size %d\n", frame_size);
		dev_kfree_skb_irq(skb);
		return true;
	}

	if (vnt_rx_data(priv, skb, frame_size))
		return true;

	dev_kfree_skb_irq(skb);

	return true;
}
