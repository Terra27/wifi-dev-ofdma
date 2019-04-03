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

#ifndef CONSTANT_WIFI_ACK_POLICY_SELECTOR_H
#define CONSTANT_WIFI_ACK_POLICY_SELECTOR_H

#include "wifi-ack-policy-selector.h"

namespace ns3 {

/**
 * \ingroup wifi
 *
 * A constant ack policy selector operating based on the values of its attributes.
 */
class ConstantWifiAckPolicySelector : public WifiAckPolicySelector
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  ConstantWifiAckPolicySelector ();
  virtual ~ConstantWifiAckPolicySelector();

  /**
   * Update the transmission parameters related to the acknowledgment policy for
   * the given PSDU. This method is typically called by the MPDU aggregator when
   * trying to aggregate another MPDU to the current A-MPDU. In fact, the
   * AckPolicySelector may switch to a different acknowledgment policy when a
   * new MPDU is aggregated to an A-MPDU.
   * Note that multi-TID A-MPDUs are currently not supported by this method.
   *
   * \param psdu the given PSDU.
   * \param params the MacLow parameters to update.
   */
  virtual void UpdateTxParams (Ptr<WifiPsdu> psdu, MacLowTransmissionParameters & params);

  /**
   * Update the transmission parameters related to the acknowledgment policy for
   * the given PSDU map. This method is typically called by MacLow once the PPDU
   * to be forwarded down to the PHY has been built, in order to update the
   * transmission parameters. This may be necessary, e.g., in case the transmission
   * parameters have been received from an OFDMA Manager but MacLow was unable to
   * find frames to send to all the receivers or an immediate acknowledgment of the
   * MU PPDU is not needed based on the policy configured through this object.
   * Note that multi-TID A-MPDUs are currently not supported by this method.
   *
   * \param psdu the given PSDU map.
   * \param params the MacLow parameters to update.
   */
  virtual void UpdateTxParams (std::map <uint16_t, Ptr<WifiPsdu>> psduMap, MacLowTransmissionParameters & params);

  /**
   * Get the acknowledgment procedure for DL MU transmissions.
   *
   * \return the acknowledgment procedure for DL MU transmissions
   */
  virtual DlMuAckSequenceType GetAckSequenceForDlMu (void) const;

  /**
   * Get the acknowledgment procedure for UL MU transmissions.
   *
   * \return the acknowledgment procedure for UL MU transmissions
   */
  virtual UlMuAckSequenceType GetAckSequenceForUlMu (void) const;

private:
  /**
   * Determine whether the given PSDU requires an immediate response (Normal Ack,
   * Block Ack or Block Ack Request followed by Block Ack) based on the given
   * transmission parameters and the sequence numbers of the MPDUs included in
   * the PSDU.
   *
   * \param psdu the given PSDU.
   * \param params the given MacLow parameters.
   * \return true if the given PSDU requires an immediate response
   */
  bool IsResponseNeeded (Ptr<WifiPsdu> psdu, const MacLowTransmissionParameters & params) const;

  bool m_useExplicitBar;              //!< true for sending BARs, false for using Implicit BAR Ack policy
  double m_baThreshold;               //!< Threshold to determine when a Block Ack must be requested
  DlMuAckSequenceType m_dlAckSeqType; //!< Type of the ack sequence used for HE MU PPDUs (DL OFDMA)
  UlMuAckSequenceType m_ulAckSeqType; //!< Type of the ack sequence used for HE TB PPDUs (UL OFDMA)
};

} //namespace ns3

#endif /* CONSTANT_WIFI_ACK_POLICY_SELECTOR_H */
