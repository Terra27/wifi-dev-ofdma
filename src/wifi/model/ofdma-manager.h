/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Universita' degli Studi di Napoli Federico II
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#ifndef OFDMA_MANAGER_H
#define OFDMA_MANAGER_H

#include <map>
#include "ns3/object.h"
#include "ns3/mac48-address.h"
#include "he-ru.h"
#include "ctrl-headers.h"
#include "mu-tx-ack-types.h"
#include "qos-utils.h"
#include "ap-wifi-mac.h"
#include "wifi-mac-queue.h"
#include "mac-low.h"
#include "wifi-remote-station-manager.h"

namespace ns3 {

/**
 * \ingroup wifi
 *
 * OfdmaManager is an abstract base class defining the API that High Efficiency
 * (HE) APs can use to determine the format of their next transmission (DL OFDMA,
 * UL OFDMA, non-OFDMA) and to get the information required to prepare the next
 * PPDU in case of DL OFDMA or UL OFDMA.
 */
class OfdmaManager : public Object
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual ~OfdmaManager ();

  /// Per station information to be provided in case of DL OFDMA transmission
  struct DlPerStaInfo
  {
    uint16_t aid;                //!< association ID
    uint8_t tid;                 //!< TID
  };

  /// Information to be provided in case of DL OFDMA transmission
  struct DlOfdmaInfo
  {
    std::map<Mac48Address,DlPerStaInfo> staInfo; //!< per-station info
    WifiTxVector txVector;                       //!< TX vector
    MacLowTransmissionParameters params;         //!< TX params
    CtrlTriggerHeader trigger;                   //!< Trigger frame (MU-BAR variant) used for acknowledgment
    Ptr<QosTxop> txop;                           //!< QosTxop transmitting the DL MU PPDU
  };

  /// Information to be provided in case of UL OFDMA transmission
  struct UlOfdmaInfo
  {
    MacLowTransmissionParameters params;         //!< TX params
    CtrlTriggerHeader trigger;                   //!< Trigger frame (Basic variant) used to solicit HE TB PPDUs
  };

  /**
   * Notify the OFDMA Manager that the AP gained access to the channel to
   * transmit the given MPDU and request it to determine the format of the next
   * transmission. It must be possible to derive the TID of the given MPDU.
   *
   * \param mpdu the MPDU the AP intends to transmit
   */
  void NotifyAccessGranted (Ptr<const WifiMacQueueItem> mpdu);

  /**
   * Get the format of the next transmission, as determined by the previous call
   * to NotifyAccessGranted.
   *
   * \return the format of the next transmission
   */
  OfdmaTxFormat GetTxFormat (void) const;

  /**
   * Get the information required to prepare an MU PPDU for DL OFDMA transmission.
   * Note that this method can only be called if GetTxFormat returns DL_OFDMA.
   *
   * \return the information required to prepare the PSDUs that are part of the
   *         MU PPDU to be transmitted
   */
  const DlOfdmaInfo& GetDlOfdmaInfo (void) const;

  /**
   * Get the information required to solicit an UL OFDMA transmission.
   * Note that this method can only be called if GetTxFormat returns UL_OFDMA.
   *
   * \return the information required to solicit an UL OFDMA transmission
   */
  const UlOfdmaInfo& GetUlOfdmaInfo (void) const;

protected:
  /**
   * Get the station manager attached to the AP.
   *
   * \return the station manager attached to the AP
   */
  Ptr<WifiRemoteStationManager> GetWifiRemoteStationManager (void) const;

  virtual void DoDispose (void);
  virtual void NotifyNewAggregate (void);

  Ptr<ApWifiMac> m_apMac;                         //!< the AP wifi MAC
  Ptr<const WifiMacQueueItem> m_mpdu;             //!< the MPDU the AP intends to send
  std::map<AcIndex, Ptr<QosTxop>> m_qosTxop;      //!< the EDCA functions
  Ptr<MacLow> m_low;                              //!< the MacLow

private:
  /**
   * Set the wifi MAC. Note that it must be the MAC of an HE AP.
   *
   * \param mac the AP wifi MAC
   */
  void SetWifiMac (Ptr<ApWifiMac> mac);

  /**
   * Select the format of the next transmission, assuming that the AP gained
   * access to the channel to transmit the given MPDU.
   *
   * \param mpdu the MPDU the AP intends to transmit
   * \return the format of the next transmission
   */
  virtual OfdmaTxFormat SelectTxFormat (Ptr<const WifiMacQueueItem> mpdu) = 0;

  /**
   * Compute the information required to prepare an MU PPDU for DL OFDMA transmission.
   *
   * \return the information required to prepare the PSDUs that are part of the
   *         MU PPDU to be transmitted
   */
  virtual DlOfdmaInfo ComputeDlOfdmaInfo (void) = 0;

  /**
   * Prepare the information required to solicit an UL OFDMA transmission.
   *
   * \return the information required to solicit an UL OFDMA transmission
   */
  virtual UlOfdmaInfo ComputeUlOfdmaInfo (void) = 0;

  OfdmaTxFormat m_TxFormat = NON_OFDMA;     //!< the format of next transmission
  DlOfdmaInfo m_dlInfo;                     //!< information needed to prepare an MU PPDU (DL OFDMA)
  UlOfdmaInfo m_ulInfo;                     //!< information required to solicit HE TB PPDUs (UL OFDMA)
};

} //namespace ns3

#endif /* OFDMA_MANAGER_H */
