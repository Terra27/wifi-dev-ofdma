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

#ifndef RR_OFDMA_MANAGER_H
#define RR_OFDMA_MANAGER_H

#include "ofdma-manager.h"
#include <list>

namespace ns3 {

/**
 * \ingroup wifi
 *
 * RrOfdmaManager is a simple OFDMA Manager that indicates to perform a DL OFDMA
 * transmission if the AP has frames to transmit to at least one station.
 * RrOfdmaManager assigns RUs of equal size (in terms of tones) to stations to
 * which the AP has frames to transmit belonging to the AC who gained access to the
 * channel or higher. The maximum number of stations that can be granted an RU
 * is configurable. Associated stations are served in a round robin fashion.
 */
class RrOfdmaManager : public OfdmaManager
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  RrOfdmaManager ();
  virtual ~RrOfdmaManager ();

private:
  /**
   * Select the format of the next transmission, assuming that the AP gained
   * access to the channel to transmit the given MPDU.
   *
   * \param mpdu the MPDU the AP intends to transmit
   * \return the format of the next transmission
   */
  virtual OfdmaTxFormat SelectTxFormat (Ptr<const WifiMacQueueItem> mpdu);

  /**
   * Compute the information required to prepare an MU PPDU for DL OFDMA transmission.
   *
   * \return a map including, for each receiver station, the information required
   *         to prepare the PSDU that is part of the MU PPDU to be transmitted
   */
  virtual DlOfdmaInfo ComputeDlOfdmaInfo (void);

  /**
   * Prepare the information required to solicit an UL OFDMA transmission.
   *
   * \return the information required to solicit an UL OFDMA transmission
   */
  virtual UlOfdmaInfo ComputeUlOfdmaInfo (void);

  /**
   * Given the channel bandwidth and the number of stations candidate for being
   * assigned an RU, maximize the number of candidate stations that can be assigned
   * an RU subject to the constraint that all the stations must be assigned an RU
   * of the same size (in terms of number of tones).
   *
   * \param bandwidth the channel bandwidth in MHz
   * \param nStations the number of candidate stations. On return, it is set to
   *                  the number of stations that are assigned an RU
   * \return the RU type
   */
  HeRu::RuType GetNumberAndTypeOfRus (uint16_t bandwidth, std::size_t& nStations) const;

  /**
   * Compute the TX vector and the TX params for a DL MU transmission assuming
   * the given list of receiver stations, the given RU type and the given type
   * of acknowledgment sequence.
   *
   * \param staList the list of receiver stations for the DL MU transmission
   * \param ruType the RU type
   * \param dlMuAckSequence the ack sequence type
   */
  void InitTxVectorAndParams (std::map<Mac48Address, DlPerStaInfo> staList,
                              HeRu::RuType ruType, DlMuAckSequenceType dlMuAckSequence);

  /**
   * Get a MU-BAR Trigger Frame built from the TX vector used for the DL MU PPDU
   * (i.e., responses will use the same set of RUs) and modified to ensure that
   * responses are sent at a rate not higher than the given MCS.
   *
   * \param dlMuTxVector the TX vector used for the DL MU PPDU
   * \param maxMcs the maximum MCS to use for the responses to the Trigger Frame
   */
  CtrlTriggerHeader GetTriggerFrameHeader (WifiTxVector dlMuTxVector, uint8_t maxMcs);

  uint8_t m_nStations;                                         //!< Number of stations/slots to fill
  uint16_t m_startStation;                                     //!< AID of the station to start with
  std::list<std::pair<Mac48Address, DlPerStaInfo>> m_staInfo;  //!< Info for the stations the AP has frames to send to
  WifiTxVector m_txVector;                                     //!< TX vector
  MacLowTransmissionParameters m_txParams;                     //!< TX params
  DlMuAckSequenceType m_dlMuAckSequence;                       //!< DL MU ack sequence type
  bool m_forceDlOfdma;                                         //!< return DL_OFDMA even if no DL MU PPDU was built
  uint16_t m_bw;                                               //!< for TESTING only
};

} //namespace ns3

#endif /* RR_DL_OFDMA_MANAGER_H */
