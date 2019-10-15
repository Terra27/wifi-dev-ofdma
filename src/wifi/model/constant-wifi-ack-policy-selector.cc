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

#include "ns3/log.h"
#include "constant-wifi-ack-policy-selector.h"
#include "mac-low.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ConstantWifiAckPolicySelector");

NS_OBJECT_ENSURE_REGISTERED (ConstantWifiAckPolicySelector);

TypeId ConstantWifiAckPolicySelector::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ConstantWifiAckPolicySelector")
    .SetParent<WifiAckPolicySelector> ()
    .AddConstructor<ConstantWifiAckPolicySelector> ()
    .SetGroupName("Wifi")
    .AddAttribute ("UseExplicitBar",
                   "Specify whether to send Block Ack Requests (if true) or use"
                   " Implicit Block Ack Request ack policy (if false).",
                   BooleanValue (false),
                   MakeBooleanAccessor (&ConstantWifiAckPolicySelector::m_useExplicitBar),
                   MakeBooleanChecker ())
    .AddAttribute ("BaThreshold",
                   "Immediate acknowledgment is requested upon transmission of a frame "
                   "whose sequence number is distant at least BaThreshold multiplied "
                   "by the transmit window size from the starting sequence number of "
                   "the transmit window. Set to zero to request a response for every "
                   "transmitted frame.",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&ConstantWifiAckPolicySelector::m_baThreshold),
                   MakeDoubleChecker<double> (0.0, 1.0))
    .AddAttribute ("DlAckSequenceType",
                   "Type of the acknowledgment sequence for DL OFDMA frames. "
                   "Use values of the DlAckSequences enum type.",
                   UintegerValue (DlMuAckSequenceType::DL_SU_FORMAT),
                   MakeUintegerAccessor (&ConstantWifiAckPolicySelector::m_dlAckSeqType),
                   MakeUintegerChecker<uint8_t> (0, DlMuAckSequenceType::DL_COUNT - 1))
    .AddAttribute ("UlAckSequenceType",
                   "Type of the acknowledgment sequence for UL OFDMA frames. "
                   "Use values of the UlAckSequences enum type.",
                   UintegerValue (UlMuAckSequenceType::UL_MULTI_STA_BLOCK_ACK),
                   MakeUintegerAccessor (&ConstantWifiAckPolicySelector::m_ulAckSeqType),
                   MakeUintegerChecker<uint8_t> (0, UlMuAckSequenceType::UL_COUNT - 1))
  ;
  return tid;
}

ConstantWifiAckPolicySelector::ConstantWifiAckPolicySelector ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

ConstantWifiAckPolicySelector::~ConstantWifiAckPolicySelector ()
{
  NS_LOG_FUNCTION (this);
}

bool
ConstantWifiAckPolicySelector::IsResponseNeeded (Ptr<WifiPsdu> psdu,
                                                 const MacLowTransmissionParameters & params) const
{
  std::set<uint8_t> tids = psdu->GetTids ();
  uint8_t tid = *tids.begin ();
  Mac48Address receiver = psdu->GetAddr1 ();

  // the received PSDU may be included in a DL MU PPDU and hence may belong to an AC different
  // (higher) than the one that got access to the channel and associated with this object
  Ptr<QosTxop> psduQosTxop = m_qosTxop->GetLow ()->GetEdca (tid);

  // find the maximum distance from the sequence number of an MPDU included in the
  // PSDU to the starting sequence number of the transmit window.
  uint16_t maxDistToStartingSeq = psdu->GetMaxDistFromStartingSeq (psduQosTxop->GetBaStartingSequence (receiver, tid));

  // An immediate response (Ack or Block Ack) is needed if any of the following holds:
  // * the maximum distance between the sequence number of an MPDU to transmit
  //   and the starting sequence number of the transmit window is greater than
  //   or equal to the window size multiplied by the BaThreshold
  // * no other frame belonging to this BA agreement is queued (because, in such
  //   a case, a Block Ack is not going to be requested anytime soon)
  // * this is the initial frame of a transmission opportunity and it is not
  //   protected by RTS/CTS (see Annex G.3 of IEEE 802.11-2016)
  return (maxDistToStartingSeq >= m_baThreshold * psduQosTxop->GetBaBufferSize (receiver, tid)
          || psduQosTxop->PeekNextFrame (tid, receiver) == 0
          || (m_qosTxop->GetTxopLimit ().IsStrictlyPositive ()
              && m_qosTxop->GetTxopRemaining () == m_qosTxop->GetTxopLimit ()
              && !params.MustSendRts ()));
}

void
ConstantWifiAckPolicySelector::UpdateTxParams (Ptr<WifiPsdu> psdu, MacLowTransmissionParameters & params)
{
  NS_LOG_FUNCTION (this << psdu << params);

  // do not update the TX params if this PSDU is part of an MU PPDU
  if (params.HasDlMuAckSequence () || params.HasUlMuAckSequence ())
    {
      return;
    }

  std::set<uint8_t> tids = psdu->GetTids ();

  if (tids.empty ())
    {
      NS_LOG_DEBUG ("No QoS Data frame in the PSDU");
      return;
    }

  if (tids.size () > 1)
    {
      NS_LOG_DEBUG ("Multi-TID A-MPDUs not supported");
      return;
    }

  Mac48Address receiver = psdu->GetAddr1 ();
  uint8_t tid = *tids.begin ();

  // Use Normal Ack if a BA agreement has not been established
  if (!m_qosTxop->GetBaAgreementEstablished (receiver, tid))
    {
      params.EnableAck ();
      return;
    }

  if (!IsResponseNeeded (psdu, params))
    {
      NS_LOG_DEBUG ("A response is not needed: no ack for now, use Block Ack policy");
      params.DisableAck ();
      return;
    }
  // An immediate response is needed
  if (psdu->GetNMpdus () == 1
      && psdu->GetHeader (0).GetSequenceNumber () == m_qosTxop->GetBaStartingSequence (receiver, tid))
    {
      NS_LOG_DEBUG ("Sending a single MPDU, no previous frame to ack: use Normal Ack policy");
      params.EnableAck ();
      return;
    }
  // Multiple MPDUs are being/have been sent
  if (psdu->GetNMpdus () == 1 || m_useExplicitBar)
    {
      // in case of single MPDU, there are previous unacknowledged frames, thus
      // we cannot use Implicit Block Ack Request policy, otherwise we get a
      // normal ack as response
      NS_LOG_DEBUG ("Scheduling a Block Ack Request");
      params.EnableBlockAckRequest (m_qosTxop->GetBlockAckReqType (receiver, tid),
                                    m_qosTxop->GetBlockAckType (receiver,tid));
      return;
    }
  // Implicit Block Ack Request policy
  NS_LOG_DEBUG ("Implicitly requesting a Block Ack");
  params.EnableBlockAck (m_qosTxop->GetBlockAckType (receiver, tid));
}

void
ConstantWifiAckPolicySelector::UpdateTxParams (std::map <uint16_t, Ptr<WifiPsdu>> psduMap,
                                               MacLowTransmissionParameters & params)
{
  NS_LOG_FUNCTION (this << params);

  if (!params.HasDlMuAckSequence () && psduMap.size () == 1 && psduMap.begin ()->first != SU_STA_ID)
    {
      // this is a PSDU sent in an HE TB PPDU
      if (!IsResponseNeeded (psduMap.begin ()->second, params))
        {
          NS_LOG_DEBUG ("A response is not needed: no ack for now, use Block Ack policy");
          params.DisableAck ();
        }
      return;
    }

  // If this is a single user transmission, return the params as updated by the
  // UpdateTxParams variant for SU PPDUs. If this is a DL MU PPDU not requiring
  // acknowledgment (only possible if this method is called twice), we do nothing.
  if (!params.HasDlMuAckSequence () && !params.HasUlMuAckSequence ())
    {
      if (psduMap.size () == 1)
        {
          UpdateTxParams (psduMap.begin ()->second, params);
        }
      return;
    }

  /** DL MU transmission **/
  if (params.HasDlMuAckSequence ())
    {
      // determine the stations which a response is needed from.
      std::list<Mac48Address> staList;

      for (auto& psdu : psduMap)
        {
          if (IsResponseNeeded (psdu.second, params))
            {
              staList.push_back (psdu.second->GetAddr1 ());
            }
        }

      if (staList.empty ())
        {
          NS_LOG_DEBUG ("No PSDU requires a response; use Block Ack Policy");
          params = MacLowTransmissionParameters ();
          return;
        }

      MacLowTransmissionParameters newParams;

      /* Acknowledgment in SU format */
      if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          newParams.SetDlMuAckSequenceType (DlMuAckSequenceType::DL_SU_FORMAT);
          bool normalAck = false;

          // check if an S-MPDU requiring a response is included in the DL MU PPDU
          for (auto& psdu : psduMap)
            {
              if (psdu.second->IsSingle ())
                {
                  auto listIt = std::find (staList.begin (), staList.end (), psdu.second->GetAddr1 ());
                  if (listIt != staList.end ())
                    {
                      NS_LOG_DEBUG ("Requesting a Normal Ack from " << psdu.second->GetAddr1 ());
                      NS_ASSERT (!normalAck);
                      newParams.EnableAck (psdu.second->GetAddr1 ());
                      staList.erase (listIt);
                      normalAck = true;
                      break;
                    }
                }
            }

          // otherwise, select the Implicit BAR policy for a PSDU
          if (!normalAck)
            {
              NS_LOG_DEBUG ("Requesting a Block Ack from " << staList.front ());
              newParams.EnableBlockAck (staList.front (), params.GetBlockAckType (staList.front ()));
              staList.erase (staList.begin ());
            }

          // all the other stations will receive a Block Ack Request
          for (auto& sta : staList)
            {
              newParams.EnableBlockAckRequest (sta, params.GetBlockAckRequestType (sta),
                                               params.GetBlockAckType (sta));
            }
        }
      /* Acknowledgment via a separate MU-BAR Trigger Frame */
      else if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR)
        {
          newParams.SetDlMuAckSequenceType (DlMuAckSequenceType::DL_MU_BAR);

          // we get here if staList is not empty, i.e., at least one PSDU needs a response.
          // Send the MU-BAR to all the stations anyway
          for (auto& psdu : psduMap)
            {
              Mac48Address sta = psdu.second->GetAddr1 ();
              newParams.EnableBlockAckRequest (sta, params.GetBlockAckRequestType (sta),
                                               params.GetBlockAckType (sta));
            }
        }
      /* Acknowledgment via aggregated MU-BAR Trigger Frames */
      else if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          newParams.SetDlMuAckSequenceType (DlMuAckSequenceType::DL_AGGREGATE_TF);

          // we get here if staList is not empty, i.e., at least one PSDU needs a response.
          // Aggregate a MU-BAR Trigger Frame to all the A-MPDUs anyway
          for (auto& psdu : psduMap)
            {
              Mac48Address sta = psdu.second->GetAddr1 ();
              newParams.EnableBlockAck (sta, params.GetBlockAckType (sta));
            }
        }

      params = newParams;
    }
}

DlMuAckSequenceType
ConstantWifiAckPolicySelector::GetAckSequenceForDlMu (void) const
{
  return m_dlAckSeqType;
}

UlMuAckSequenceType
ConstantWifiAckPolicySelector::GetAckSequenceForUlMu (void) const
{
  return m_ulAckSeqType;
}

} //namespace ns3
