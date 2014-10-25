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
	unsigned char byRateIdx;

	for (byRateIdx = 0; byRateIdx < MAX_RATE; byRateIdx++) {
		if (acbyRxRate[byRateIdx % MAX_RATE] == byRate)
			return byRateIdx;
	}

	return 0;
}

static void
s_vGetDASA(unsigned char *pbyRxBufferAddr, unsigned int *pcbHeaderSize,
	   PSEthernetHeader psEthHeader)
{
	unsigned int cbHeaderSize = 0;
	PS802_11Header  pMACHeader;
	int             ii;

	pMACHeader = (PS802_11Header) (pbyRxBufferAddr + cbHeaderSize);

	if ((pMACHeader->wFrameCtl & FC_TODS) == 0) {
		if (pMACHeader->wFrameCtl & FC_FROMDS) {
			for (ii = 0; ii < ETH_ALEN; ii++) {
				psEthHeader->abyDstAddr[ii] = pMACHeader->abyAddr1[ii];
				psEthHeader->abySrcAddr[ii] = pMACHeader->abyAddr3[ii];
			}
		} else {
			// IBSS mode
			for (ii = 0; ii < ETH_ALEN; ii++) {
				psEthHeader->abyDstAddr[ii] = pMACHeader->abyAddr1[ii];
				psEthHeader->abySrcAddr[ii] = pMACHeader->abyAddr2[ii];
			}
		}
	} else {
		// Is AP mode..
		if (pMACHeader->wFrameCtl & FC_FROMDS) {
			for (ii = 0; ii < ETH_ALEN; ii++) {
				psEthHeader->abyDstAddr[ii] = pMACHeader->abyAddr3[ii];
				psEthHeader->abySrcAddr[ii] = pMACHeader->abyAddr4[ii];
				cbHeaderSize += 6;
			}
		} else {
			for (ii = 0; ii < ETH_ALEN; ii++) {
				psEthHeader->abyDstAddr[ii] = pMACHeader->abyAddr3[ii];
				psEthHeader->abySrcAddr[ii] = pMACHeader->abyAddr2[ii];
			}
		}
	}
	*pcbHeaderSize = cbHeaderSize;
}

bool
device_receive_frame(
	struct vnt_private *pDevice,
	PSRxDesc pCurrRD
)
{
	PDEVICE_RD_INFO  pRDInfo = pCurrRD->pRDInfo;
	struct net_device_stats *pStats = &pDevice->dev->stats;
	struct sk_buff *skb;
	PSMgmtObject    pMgmt = pDevice->pMgmt;
	PSRxMgmtPacket  pRxPacket = &(pDevice->pMgmt->sRxPacket);
	PS802_11Header  p802_11Header;
	unsigned char *pbyRsr;
	unsigned char *pbyNewRsr;
	unsigned char *pbyRSSI;
	__le64 *pqwTSFTime;
	unsigned short *pwFrameSize;
	unsigned char *pbyFrame;
	bool bDeFragRx = false;
	bool bIsWEP = false;
	unsigned int cbHeaderOffset;
	unsigned int FrameSize;
	unsigned short wEtherType = 0;
	int             iSANodeIndex = -1;
	int             iDANodeIndex = -1;
	unsigned int ii;
	unsigned int cbIVOffset;
	bool bExtIV = false;
	unsigned char *pbyRxSts;
	unsigned char *pbyRxRate;
	unsigned char *pbySQ;
	unsigned int cbHeaderSize;
	PSKeyItem       pKey = NULL;
	unsigned short wRxTSC15_0 = 0;
	unsigned long dwRxTSC47_16 = 0;
	SKeyItem        STempKey;
	// 802.11h RPI
	unsigned long dwDuration = 0;
	long            ldBm = 0;
	long            ldBmThreshold = 0;
	PS802_11Header pMACHeader;
	bool bRxeapol_key = false;

	skb = pRDInfo->skb;

	pci_unmap_single(pDevice->pcid, pRDInfo->skb_dma,
			 pDevice->rx_buf_sz, PCI_DMA_FROMDEVICE);

	pwFrameSize = (unsigned short *)(skb->data + 2);
	FrameSize = cpu_to_le16(pCurrRD->m_rd1RD1.wReqCount) - cpu_to_le16(pCurrRD->m_rd0RD0.wResCount);

	// Max: 2312Payload + 30HD +4CRC + 2Padding + 4Len + 8TSF + 4RSR
	// Min (ACK): 10HD +4CRC + 2Padding + 4Len + 8TSF + 4RSR
	if ((FrameSize > 2364) || (FrameSize <= 32)) {
		// Frame Size error drop this packet.
		pr_debug("---------- WRONG Length 1\n");
		return false;
	}

	pbyRxSts = (unsigned char *)(skb->data);
	pbyRxRate = (unsigned char *)(skb->data + 1);
	pbyRsr = (unsigned char *)(skb->data + FrameSize - 1);
	pbyRSSI = (unsigned char *)(skb->data + FrameSize - 2);
	pbyNewRsr = (unsigned char *)(skb->data + FrameSize - 3);
	pbySQ = (unsigned char *)(skb->data + FrameSize - 4);
	pqwTSFTime = (__le64 *)(skb->data + FrameSize - 12);
	pbyFrame = (unsigned char *)(skb->data + 4);

	// get packet size
	FrameSize = cpu_to_le16(*pwFrameSize);

	if ((FrameSize > 2346)|(FrameSize < 14)) { // Max: 2312Payload + 30HD +4CRC
		// Min: 14 bytes ACK
		pr_debug("---------- WRONG Length 2\n");
		return false;
	}

	// update receive statistic counter
	STAvUpdateRDStatCounter(&pDevice->scStatistic,
				*pbyRsr,
				*pbyNewRsr,
				*pbyRxRate,
				pbyFrame,
				FrameSize);

	pMACHeader = (PS802_11Header)((unsigned char *)(skb->data) + 8);

	if (pDevice->bMeasureInProgress) {
		if ((*pbyRsr & RSR_CRCOK) != 0)
			pDevice->byBasicMap |= 0x01;

		dwDuration = FrameSize << 4;
		dwDuration /= acbyRxRate[*pbyRxRate%MAX_RATE];
		if (*pbyRxRate <= RATE_11M) {
			if (*pbyRxSts & 0x01) {
				// long preamble
				dwDuration += 192;
			} else {
				// short preamble
				dwDuration += 96;
			}
		} else {
			dwDuration += 16;
		}
		RFvRSSITodBm(pDevice, *pbyRSSI, &ldBm);
		ldBmThreshold = -57;
		for (ii = 7; ii > 0;) {
			if (ldBm > ldBmThreshold)
				break;

			ldBmThreshold -= 5;
			ii--;
		}
		pDevice->dwRPIs[ii] += dwDuration;
		return false;
	}

	if (!is_multicast_ether_addr(pbyFrame)) {
		if (WCTLbIsDuplicate(&(pDevice->sDupRxCache), (PS802_11Header)(skb->data + 4))) {
			pDevice->s802_11Counter.FrameDuplicateCount++;
			return false;
		}
	}

	// Use for TKIP MIC
	s_vGetDASA(skb->data+4, &cbHeaderSize, &pDevice->sRxEthHeader);

	// filter packet send from myself
	if (ether_addr_equal(pDevice->sRxEthHeader.abySrcAddr,
			     pDevice->abyCurrentNetAddr))
		return false;

	if ((pMgmt->eCurrMode == WMAC_MODE_ESS_AP) || (pMgmt->eCurrMode == WMAC_MODE_IBSS_STA)) {
		if (IS_CTL_PSPOLL(pbyFrame) || !IS_TYPE_CONTROL(pbyFrame)) {
			p802_11Header = (PS802_11Header)(pbyFrame);
			// get SA NodeIndex
			if (BSSDBbIsSTAInNodeDB(pMgmt, (unsigned char *)(p802_11Header->abyAddr2), &iSANodeIndex)) {
				pMgmt->sNodeDBTable[iSANodeIndex].ulLastRxJiffer = jiffies;
				pMgmt->sNodeDBTable[iSANodeIndex].uInActiveCount = 0;
			}
		}
	}

	if (pMgmt->eCurrMode == WMAC_MODE_ESS_AP) {
		if (s_bAPModeRxCtl(pDevice, pbyFrame, iSANodeIndex))
			return false;
	}

	if (IS_FC_WEP(pbyFrame)) {
		bool bRxDecryOK = false;

		pr_debug("rx WEP pkt\n");
		bIsWEP = true;
		if ((pDevice->bEnableHostWEP) && (iSANodeIndex >= 0)) {
			pKey = &STempKey;
			pKey->byCipherSuite = pMgmt->sNodeDBTable[iSANodeIndex].byCipherSuite;
			pKey->dwKeyIndex = pMgmt->sNodeDBTable[iSANodeIndex].dwKeyIndex;
			pKey->uKeyLength = pMgmt->sNodeDBTable[iSANodeIndex].uWepKeyLength;
			pKey->dwTSC47_16 = pMgmt->sNodeDBTable[iSANodeIndex].dwTSC47_16;
			pKey->wTSC15_0 = pMgmt->sNodeDBTable[iSANodeIndex].wTSC15_0;
			memcpy(pKey->abyKey,
			       &pMgmt->sNodeDBTable[iSANodeIndex].abyWepKey[0],
			       pKey->uKeyLength
);

			bRxDecryOK = s_bHostWepRxEncryption(pDevice,
							    pbyFrame,
							    FrameSize,
							    pbyRsr,
							    pMgmt->sNodeDBTable[iSANodeIndex].bOnFly,
							    pKey,
							    pbyNewRsr,
							    &bExtIV,
							    &wRxTSC15_0,
							    &dwRxTSC47_16);
		} else {
			bRxDecryOK = s_bHandleRxEncryption(pDevice,
							   pbyFrame,
							   FrameSize,
							   pbyRsr,
							   pbyNewRsr,
							   &pKey,
							   &bExtIV,
							   &wRxTSC15_0,
							   &dwRxTSC47_16);
		}

		if (bRxDecryOK) {
			if ((*pbyNewRsr & NEWRSR_DECRYPTOK) == 0) {
				pr_debug("ICV Fail\n");
				if ((pDevice->pMgmt->eAuthenMode == WMAC_AUTH_WPA) ||
				    (pDevice->pMgmt->eAuthenMode == WMAC_AUTH_WPAPSK) ||
				    (pDevice->pMgmt->eAuthenMode == WMAC_AUTH_WPANONE) ||
				    (pDevice->pMgmt->eAuthenMode == WMAC_AUTH_WPA2) ||
				    (pDevice->pMgmt->eAuthenMode == WMAC_AUTH_WPA2PSK)) {
					if ((pKey != NULL) && (pKey->byCipherSuite == KEY_CTL_TKIP))
						pDevice->s802_11Counter.TKIPICVErrors++;
					else if ((pKey != NULL) && (pKey->byCipherSuite == KEY_CTL_CCMP))
						pDevice->s802_11Counter.CCMPDecryptErrors++;
				}
				return false;
			}
		} else {
			pr_debug("WEP Func Fail\n");
			return false;
		}
		if ((pKey != NULL) && (pKey->byCipherSuite == KEY_CTL_CCMP))
			FrameSize -= 8;         // Message Integrity Code
		else
			FrameSize -= 4;         // 4 is ICV
	}

	//
	// RX OK
	//
	//remove the CRC length
	FrameSize -= ETH_FCS_LEN;

	if ((!(*pbyRsr & (RSR_ADDRBROAD | RSR_ADDRMULTI))) && // unicast address
	    (IS_FRAGMENT_PKT((skb->data+4)))
) {
		// defragment
		bDeFragRx = WCTLbHandleFragment(pDevice, (PS802_11Header)(skb->data+4), FrameSize, bIsWEP, bExtIV);
		pDevice->s802_11Counter.ReceivedFragmentCount++;
		if (bDeFragRx) {
			// defrag complete
			skb = pDevice->sRxDFCB[pDevice->uCurrentDFCBIdx].skb;
			FrameSize = pDevice->sRxDFCB[pDevice->uCurrentDFCBIdx].cbFrameLength;

		} else {
			return false;
		}
	}

// Management & Control frame Handle
	if ((IS_TYPE_DATA((skb->data+4))) == false) {
		// Handle Control & Manage Frame

		if (IS_TYPE_MGMT((skb->data+4))) {
			unsigned char *pbyData1;
			unsigned char *pbyData2;

			pRxPacket->p80211Header = (PUWLAN_80211HDR)(skb->data+4);
			pRxPacket->cbMPDULen = FrameSize;
			pRxPacket->uRSSI = *pbyRSSI;
			pRxPacket->bySQ = *pbySQ;
			pRxPacket->qwLocalTSF = le64_to_cpu(*pqwTSFTime);
			if (bIsWEP) {
				// strip IV
				pbyData1 = WLAN_HDR_A3_DATA_PTR(skb->data+4);
				pbyData2 = WLAN_HDR_A3_DATA_PTR(skb->data+4) + 4;
				for (ii = 0; ii < (FrameSize - 4); ii++) {
					*pbyData1 = *pbyData2;
					pbyData1++;
					pbyData2++;
				}
			}
			pRxPacket->byRxRate = s_byGetRateIdx(*pbyRxRate);
			pRxPacket->byRxChannel = (*pbyRxSts) >> 2;

			vMgrRxManagePacket((void *)pDevice, pDevice->pMgmt, pRxPacket);

			// hostap Deamon handle 802.11 management
			if (pDevice->bEnableHostapd) {
				skb->dev = pDevice->apdev;
				skb->data += 4;
				skb->tail += 4;
				skb_put(skb, FrameSize);
				skb_reset_mac_header(skb);
				skb->pkt_type = PACKET_OTHERHOST;
				skb->protocol = htons(ETH_P_802_2);
				memset(skb->cb, 0, sizeof(skb->cb));
				netif_rx(skb);
				return true;
			}
		}

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
